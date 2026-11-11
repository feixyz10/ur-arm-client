"""Verify that semantic values are handled at the RTDE boundary."""

from unittest.mock import patch

import pytest

from ur_arm_client import JPos, JVel, OperationError, Pose, Twist, URArmClient, Wrench

JOINTS = JPos(0, -1.57, 1.57, 0, 0, 0)
POSE = Pose((0.5, 0.2, 0.3), (0.4, 0.1, 0.0))
JOINT_VELOCITY = JVel.from_sequence(JOINTS)
TWIST = Twist((0.01, 0.02, 0.03), (0.04, 0.05, 0.06))
WRENCH = Wrench((1, 2, 3), (4, 5, 6))


@pytest.fixture
def client_and_control():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        yield client, control.return_value


@pytest.mark.parametrize(
    "move_name, rtde_name, target",
    [
        ("movej", "moveJ", JOINTS),
        ("movej", "moveJ", tuple(JOINTS)),
        ("movel", "moveL", POSE),
        ("movel", "moveL", tuple(POSE)),
    ],
)
def test_absolute_moves_send_plain_rtde_vectors(
    client_and_control, move_name, rtde_name, target
):
    client, control = client_and_control
    getattr(client, move_name)(target)
    getattr(control, rtde_name).assert_called_once_with(list(target), 0.25, 1.0, False)


@pytest.mark.parametrize(
    "path_type, rtde_name, target", [("j", "moveJ", JOINTS), ("l", "moveL", POSE)]
)
def test_paths_accept_matching_values(client_and_control, path_type, rtde_name, target):
    client, control = client_and_control
    client.move_path((target, tuple(target)), path_type=path_type, blend_radius=0.01)
    getattr(control, rtde_name).assert_called_once_with(
        [[*target, 0.25, 1.0, 0.01], [*target, 0.25, 1.0, 0.0]], False
    )


@pytest.mark.parametrize(
    "method, target, options",
    [
        ("movej", POSE, {}),
        ("movej", JOINT_VELOCITY, {}),
        ("movel", TWIST, {}),
        ("movel", WRENCH, {}),
        ("fk", JOINT_VELOCITY, {}),
        ("ik", POSE, {"qnear": JOINT_VELOCITY}),
        ("ik_check", WRENCH, {}),
        ("move_path", [TWIST], {"path_type": "l"}),
        ("movel_relative", TWIST, {}),
        ("movel", JOINTS, {}),
        ("fk", POSE, {}),
        ("move_path", [POSE], {"path_type": "j"}),
        ("move_path", [JOINTS], {"path_type": "l"}),
        ("movej_relative", JOINTS, {}),
        ("movel_relative", POSE, {}),
    ],
)
def test_rejects_semantically_wrong_targets(
    client_and_control, method, target, options
):
    client, control = client_and_control
    with pytest.raises(OperationError):
        getattr(client, method)(target, **options)
    control.moveJ.assert_not_called()
    control.moveL.assert_not_called()
    control.getForwardKinematics.assert_not_called()
    control.getInverseKinematics.assert_not_called()
    control.getInverseKinematicsHasSolution.assert_not_called()
    control.poseTrans.assert_not_called()
