"""Tests for path movement (move_path)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError, OperationError


def test_move_path_with_joint_space_waypoints():
    """Test that move_path executes joint space path with type 'j'."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        waypoints = [
            [0.0, -1.0, 1.0, 0.0, 0.0, 0.0],
            [0.1, -0.9, 1.0, 0.0, 0.0, 0.0],
            [0.2, -0.8, 1.0, 0.0, 0.0, 0.0],
        ]

        result = driver.move_path(
            waypoints, path_type="j", velocity=0.8, acceleration=1.0
        )

        assert result is None
        mock_ctrl_instance.moveJ.assert_called_once_with(waypoints, 0.8, 1.0, True)


def test_move_path_with_cartesian_space_waypoints():
    """Test that move_path executes Cartesian path with type 'l'."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        waypoints = [
            [0.3, 0.2, 0.5, 0.0, 0.0, 0.0],
            [0.35, 0.25, 0.55, 0.0, 0.0, 0.0],
        ]

        result = driver.move_path(waypoints, path_type="l")

        assert result is None
        # Should use default linear velocity/acceleration
        mock_ctrl_instance.moveL.assert_called_once_with(waypoints, 0.25, 1.2, True)


def test_move_path_validates_path_type():
    """Test that move_path raises ValueError for invalid path_type."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        waypoints = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        with pytest.raises(ValueError, match="path_type must be 'j' or 'l', got 'x'"):
            driver.move_path(waypoints, path_type="x")
