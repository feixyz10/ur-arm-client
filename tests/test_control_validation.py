"""Invalid control inputs are rejected before reaching the native interface."""

from unittest.mock import patch

import pytest

from ur_arm_client import OperationError, URArmClient


@pytest.fixture
def client_and_control():
    with (
        patch("ur_arm_client._driver.RTDEController") as control,
        patch("ur_arm_client._driver.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        yield client, control.return_value


@pytest.mark.parametrize(("method", "native"), [("movej", "moveJ"), ("movel", "moveL")])
def test_rejects_non_finite_target_and_invalid_motion_rates(
    client_and_control, method, native
):
    client, control = client_and_control
    move = getattr(client, method)
    for target, options in (
        ([0.0] * 5 + [float("nan")], {}),
        ([0.0] * 6, {"velocity": 0}),
        ([0.0] * 6, {"acceleration": float("inf")}),
    ):
        with pytest.raises(OperationError):
            move(target, **options)
    getattr(control, native).assert_not_called()


@pytest.mark.parametrize("joints", [[], [0.0] * 5, [0.0] * 5 + [float("inf")]])
def test_fk_requires_explicit_finite_joint_target(client_and_control, joints):
    client, control = client_and_control
    with pytest.raises(OperationError, match="6 finite joint positions"):
        client.fk(joints)
    control.getForwardKinematics.assert_not_called()


@pytest.mark.parametrize(("method", "native"), [("stopj", "stopJ"), ("stopl", "stopL")])
def test_rejects_invalid_stop_deceleration(client_and_control, method, native):
    client, control = client_and_control
    with pytest.raises(OperationError, match="deceleration"):
        getattr(client, method)(-1)
    getattr(control, native).assert_not_called()


@pytest.mark.parametrize("method", ["fk", "stopj", "stopl"])
def test_control_only_methods_report_read_only_mode(method):
    args = ([0.0] * 6,) if method == "fk" else ()
    with (
        patch("ur_arm_client._driver.RTDEController") as control,
        patch("ur_arm_client._driver.RTDEReceiver"),
        URArmClient("robot", read_only=True) as client,
    ):
        with pytest.raises(OperationError, match="read-only mode"):
            getattr(client, method)(*args)
        control.assert_not_called()


def test_idle_wait_still_validates_timeout(client_and_control):
    client, control = client_and_control
    with pytest.raises(ValueError, match="Timeout"):
        client.wait_all_done(timeout=-1)
    control.getAsyncOperationProgressEx.assert_not_called()
