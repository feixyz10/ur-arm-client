"""Tests for relative movements (movej_relative, movel_relative)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver


def test_movej_relative_adds_delta_to_current_joint_positions():
    """Test that movej_relative adds delta to current joint positions."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        # Current joint positions
        mock_recv_instance.getActualQ.return_value = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        delta = [0.1, 0.2, -0.3, 0.0, 0.0, 0.0]
        driver.movej_relative(delta, velocity=0.8, acceleration=1.0, blocking=True)

        # Should add delta to current position
        expected_target = [0.1, -0.8, 0.7, 0.0, 0.0, 0.0]
        mock_ctrl_instance.moveJ.assert_called_once_with(
            expected_target, 0.8, 1.0, True
        )


def test_movel_relative_adds_delta_to_current_pose():
    """Test that movel_relative adds delta to current TCP pose."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        # Current TCP pose
        mock_recv_instance.getActualTCPPose.return_value = [
            0.3,
            0.2,
            0.5,
            0.0,
            0.0,
            0.0,
        ]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        delta = [0.01, -0.02, 0.05, 0.0, 0.0, 0.0]
        driver.movel_relative(delta, velocity=0.15, acceleration=0.8, blocking=True)

        # Should add delta to current pose
        assert mock_ctrl_instance.moveL.called
        call_args = mock_ctrl_instance.moveL.call_args
        target = call_args[0][0]
        velocity = call_args[0][1]
        acceleration = call_args[0][2]
        blocking = call_args[0][3]

        # Check parameters
        assert velocity == 0.15
        assert acceleration == 0.8
        assert blocking is True
        # Check target is approximately correct (floating point tolerance)
        assert len(target) == 6
        assert abs(target[0] - 0.31) < 1e-10  # 0.3 + 0.01
        assert abs(target[1] - 0.18) < 1e-10  # 0.2 - 0.02
        assert abs(target[2] - 0.55) < 1e-10  # 0.5 + 0.05
        assert target[3] == 0.0
        assert target[4] == 0.0
        assert target[5] == 0.0


def test_movej_relative_uses_default_parameters():
    """Test that movej_relative uses default velocity and acceleration."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.movej_relative([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Should use defaults from movej
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 1.05, 1.4, True
        )
