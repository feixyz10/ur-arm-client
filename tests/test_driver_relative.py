"""Tests for relative movements (movej_relative, movel_relative)."""

import pytest
from unittest.mock import Mock, patch

from ur_arm_client import OperationError, URArmClient


def test_movej_relative_adds_delta_to_current_joint_positions():
    """Test that movej_relative adds delta to current joint positions."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        # Current joint positions
        mock_recv_instance.getActualQ.return_value = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        delta = [0.1, 0.2, -0.3, 0.0, 0.0, 0.0]
        driver.movej_relative(delta, velocity=0.8, acceleration=1.0, blocking=True)

        # Should add delta to current position
        expected_target = [0.1, -0.8, 0.7, 0.0, 0.0, 0.0]
        mock_ctrl_instance.moveJ.assert_called_once_with(
            expected_target, 0.8, 1.0, False
        )


def test_movel_relative_composes_offset_in_tool_frame():
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
        URArmClient("robot") as client,
    ):
        controller = mock_ctrl.return_value
        pose = [0.3, 0.2, 0.5, 0, 0, 1.57079632679]
        delta = [0.1, 0, 0, 0, 0, 0]
        target = [0.3, 0.3, 0.5, 0, 0, 1.57079632679]
        mock_recv.return_value.getActualTCPPose.return_value = pose
        controller.poseTrans.return_value = target

        client.movel_relative(delta, velocity=0.15, acceleration=0.8)

        controller.poseTrans.assert_called_once_with(pose, delta)
        controller.moveL.assert_called_once_with(target, 0.15, 0.8, False)


@pytest.mark.parametrize("method", ["movej_relative", "movel_relative"])
def test_relative_delta_must_have_six_finite_values(method):
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        for delta in ([0] * 5, [0] * 7, [0] * 5 + [float("nan")]):
            with pytest.raises(OperationError, match="6 finite delta values"):
                getattr(client, method)(delta)
        control.return_value.moveJ.assert_not_called()
        control.return_value.moveL.assert_not_called()


def test_movej_relative_uses_default_parameters():
    """Test that movej_relative uses default velocity and acceleration."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.movej_relative([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Should use defaults from movej
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 0.25, 1.0, False
        )
