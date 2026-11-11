"""Inverse kinematics results and failure handling at the RTDE boundary."""

from unittest.mock import patch

import pytest

from ur_arm_client import (
    ConnectionError,
    JPos,
    OperationError,
    Pose,
    URArmClient,
)

POSE = Pose((0.5, 0.2, 0.3), (0.4, 0.1, 0.0))
JOINTS = JPos(0, -1.57, 1.57, 0, 0, 0)


@pytest.fixture
def firmware_probe():
    with patch(
        "ur_arm_client._client._read_firmware", return_value=(5, 10, 0, 1)
    ) as probe:
        yield probe


@pytest.fixture
def client_and_controller(firmware_probe):
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        controller = control.return_value
        controller.isConnected.return_value = True
        controller.isProgramRunning.return_value = True
        yield client, controller


@pytest.mark.parametrize(
    ("target", "reference"),
    [(POSE, JOINTS), (tuple(POSE), list(JOINTS)), (list(POSE), None)],
)
def test_ik_solves_once_and_returns_joint_positions(
    client_and_controller, firmware_probe, target, reference
):
    client, controller = client_and_controller
    controller.getInverseKinematics.return_value = list(JOINTS)

    assert client.ik(target, qnear=reference) == JOINTS

    controller.getInverseKinematics.assert_called_once_with(
        list(POSE), [] if reference is None else list(reference)
    )
    controller.getInverseKinematicsHasSolution.assert_not_called()
    controller.isProgramRunning.assert_not_called()
    firmware_probe.assert_not_called()
    controller.moveJ.assert_not_called()
    controller.moveL.assert_not_called()


@pytest.mark.parametrize("has_solution", [False, True])
def test_ik_check_only_checks(client_and_controller, has_solution):
    client, controller = client_and_controller
    controller.getInverseKinematicsHasSolution.return_value = has_solution

    assert client.ik_check(POSE, qnear=JOINTS) is has_solution

    controller.getInverseKinematicsHasSolution.assert_called_once_with(
        list(POSE), list(JOINTS)
    )
    controller.getInverseKinematics.assert_not_called()
    controller.moveJ.assert_not_called()
    controller.moveL.assert_not_called()


def test_empty_result_is_checked_after_solve(client_and_controller):
    client, controller = client_and_controller
    controller.getInverseKinematicsHasSolution.return_value = False

    def solve(*_):
        controller.getInverseKinematicsHasSolution.assert_not_called()
        return []

    controller.getInverseKinematics.side_effect = solve

    assert client.ik(POSE, qnear=JOINTS) is None

    controller.getInverseKinematics.assert_called_once_with(list(POSE), list(JOINTS))
    controller.getInverseKinematicsHasSolution.assert_called_once_with(
        list(POSE), list(JOINTS)
    )


def test_empty_result_when_a_solution_exists_is_an_error(client_and_controller):
    client, controller = client_and_controller
    controller.getInverseKinematics.return_value = []
    controller.getInverseKinematicsHasSolution.return_value = True

    with pytest.raises(OperationError, match="despite a solution"):
        client.ik(POSE)


def test_empty_result_after_disconnect_is_not_no_solution(client_and_controller):
    client, controller = client_and_controller

    def solve(*_):
        controller.isConnected.return_value = False
        return []

    controller.getInverseKinematics.side_effect = solve

    with pytest.raises(ConnectionError, match="Connection lost"):
        client.ik(POSE)
    controller.getInverseKinematicsHasSolution.assert_not_called()


def test_empty_result_when_control_script_stops_is_not_no_solution(
    client_and_controller,
):
    client, controller = client_and_controller
    controller.getInverseKinematics.return_value = []
    controller.isProgramRunning.return_value = False

    with pytest.raises(OperationError, match="control script stopped"):
        client.ik(POSE)
    controller.getInverseKinematicsHasSolution.assert_not_called()


@pytest.mark.parametrize("result", [[0.0], [float("nan")] * 6])
def test_invalid_native_result_is_not_no_solution(client_and_controller, result):
    client, controller = client_and_controller
    controller.getInverseKinematics.return_value = result

    with pytest.raises(OperationError, match="invalid inverse kinematics data"):
        client.ik(POSE)
    controller.getInverseKinematicsHasSolution.assert_not_called()


@pytest.mark.parametrize(
    ("method", "native"),
    [("ik", "getInverseKinematics"), ("ik_check", "getInverseKinematicsHasSolution")],
)
def test_native_errors_propagate(client_and_controller, method, native):
    client, controller = client_and_controller
    getattr(controller, native).side_effect = RuntimeError("RTDE command failed")

    with pytest.raises(RuntimeError, match="RTDE command failed"):
        getattr(client, method)(POSE)


def test_confirmation_error_is_not_no_solution(client_and_controller):
    client, controller = client_and_controller
    controller.getInverseKinematics.return_value = []
    controller.getInverseKinematicsHasSolution.side_effect = RuntimeError(
        "RTDE command failed"
    )

    with pytest.raises(RuntimeError, match="RTDE command failed"):
        client.ik(POSE)


@pytest.mark.parametrize("method", ["ik", "ik_check"])
@pytest.mark.parametrize(
    ("target", "reference"),
    [
        (JOINTS, None),
        ([], None),
        ([0.0] * 5 + [float("inf")], None),
        (POSE, POSE),
        (POSE, []),
        (POSE, [0.0] * 5 + [float("nan")]),
    ],
)
def test_invalid_arguments_do_not_reach_controller(
    client_and_controller, method, target, reference
):
    client, controller = client_and_controller

    with pytest.raises(OperationError):
        getattr(client, method)(target, qnear=reference)
    controller.getInverseKinematics.assert_not_called()
    controller.getInverseKinematicsHasSolution.assert_not_called()


@pytest.mark.parametrize("method", ["ik", "ik_check"])
def test_ik_requires_control_connection(method):
    client = URArmClient("robot")
    with pytest.raises(ConnectionError, match="Not connected"):
        getattr(client, method)(POSE)

    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot", read_only=True) as read_only,
    ):
        with pytest.raises(OperationError, match="read-only"):
            getattr(read_only, method)(POSE)
        control.assert_not_called()


@pytest.mark.parametrize("version", [(3, 14, 9, 1), (5, 9, 9, 1)])
def test_unsupported_firmware_cannot_confirm_a_solution(
    client_and_controller, firmware_probe, version
):
    client, controller = client_and_controller
    firmware_probe.return_value = version
    controller.getInverseKinematics.return_value = []

    with pytest.raises(OperationError, match="firmware"):
        client.ik_check(POSE)
    with pytest.raises(OperationError, match="firmware"):
        client.ik(POSE)

    firmware_probe.assert_called_once_with("robot")
    controller.getInverseKinematics.assert_called_once()
    controller.getInverseKinematicsHasSolution.assert_not_called()


@pytest.mark.parametrize("version", [(3, 15, 0, 1), (5, 10, 0, 1), (10, 0, 0, 1)])
def test_supported_firmware_probe_is_shared_with_postcheck(
    client_and_controller, firmware_probe, version
):
    client, controller = client_and_controller
    firmware_probe.return_value = version
    controller.getInverseKinematics.return_value = []
    controller.getInverseKinematicsHasSolution.return_value = False

    assert client.ik_check(POSE) is False
    assert client.ik(POSE) is None

    firmware_probe.assert_called_once_with("robot")
    assert controller.getInverseKinematicsHasSolution.call_count == 2


def test_reconnect_refreshes_firmware_for_the_new_host(
    client_and_controller, firmware_probe
):
    client, controller = client_and_controller
    controller.getInverseKinematicsHasSolution.return_value = True
    firmware_probe.side_effect = [(5, 10, 0, 1), (5, 9, 0, 1)]
    client.ip = "other robot"

    assert client.ik_check(POSE) is True
    firmware_probe.assert_called_once_with("robot")

    client.disconnect()
    client.connect()
    with pytest.raises(OperationError, match="firmware 5.9.0"):
        client.ik_check(POSE)

    assert firmware_probe.call_args_list[1].args == ("other robot",)
    controller.getInverseKinematicsHasSolution.assert_called_once()


@pytest.mark.parametrize("method", ["ik_check", "ik"])
def test_unknown_firmware_does_not_allow_solubility_checks(
    client_and_controller, firmware_probe, method
):
    client, controller = client_and_controller
    controller.getInverseKinematics.return_value = []
    firmware_probe.side_effect = ConnectionError("Could not determine firmware")

    with pytest.raises(ConnectionError, match="Could not determine firmware"):
        getattr(client, method)(POSE)
    controller.getInverseKinematicsHasSolution.assert_not_called()
