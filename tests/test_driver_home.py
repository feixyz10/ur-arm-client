"""Tests for home position functionality (set_home, go_home)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError, OperationError


def test_set_home_stores_pose():
    """Test that set_home stores the home pose."""
    driver = URDriver(ip="192.168.1.10", lazy=True)
    home_pose = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
    driver.set_home(home_pose)
    assert driver.home_pose == home_pose


def test_set_home_with_none():
    """Test that set_home can set home to None."""
    driver = URDriver(ip="192.168.1.10", lazy=True)
    driver.set_home(None)
    assert driver.home_pose is None


def test_go_home_with_custom_pose():
    """Test that go_home uses custom home pose when set."""
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

        home_pose = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        driver.set_home(home_pose)

        result = driver.go_home(velocity=0.8, acceleration=1.0)

        assert result is None  # blocking=True returns None
        mock_ctrl_instance.moveJ.assert_called_once_with(home_pose, 0.8, 1.0, True)


def test_go_home_defaults_to_zero_position():
    """Test that go_home defaults to zero position when no home is set."""
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

        result = driver.go_home()

        assert result is None  # blocking=True returns None
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, 0, 0, 0, 0, 0], 1.05, 1.4, True
        )


def test_go_home_requires_connection():
    """Test that go_home raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)
    driver.set_home([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        driver.go_home()
