"""Tests for movel (Cartesian linear movement)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError, OperationError


def test_movel_blocking_returns_none():
    """Test that blocking movel returns None on success."""
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

        result = driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0], blocking=True)

        assert result is None
        mock_ctrl_instance.moveL.assert_called_once_with(
            [0.5, 0.2, 0.3, 0.0, 0.0, 0.0], 0.25, 1.2, True
        )


def test_movel_non_blocking_returns_token():
    """Test that non-blocking movel returns MotionToken."""
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

        result = driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0], blocking=False)

        assert result is not None
        assert result.movement_type == "l"
        assert result.target == [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        assert result.velocity == 0.25
        assert result.acceleration == 1.2
        assert result.blocking is False
        mock_ctrl_instance.moveL.assert_called_once_with(
            [0.5, 0.2, 0.3, 0.0, 0.0, 0.0], 0.25, 1.2, False
        )


def test_movel_custom_parameters():
    """Test movel with custom velocity and acceleration."""
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

        driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0], velocity=0.1, acceleration=0.5)

        mock_ctrl_instance.moveL.assert_called_once_with(
            [0.5, 0.2, 0.3, 0.0, 0.0, 0.0], 0.1, 0.5, True
        )


def test_movel_requires_6_pose_values():
    """Test that movel raises error for incorrect number of pose values."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        with pytest.raises(OperationError, match="movel requires 6 pose values, got 5"):
            driver.movel([0.5, 0.2, 0.3, 0.0, 0.0])

        with pytest.raises(OperationError, match="movel requires 6 pose values, got 7"):
            driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0])


def test_movel_requires_connection():
    """Test that movel raises ConnectionError when not connected."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Not connected to robot"):
            driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0])


def test_movel_raises_on_failure():
    """Test that movel raises OperationError when controller fails."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = False
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        with pytest.raises(OperationError, match="movel failed"):
            driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0])


def test_movel_raises_in_read_only_mode():
    """Test that movel raises error in read-only mode."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        with pytest.raises(OperationError, match="Cannot move in read-only mode"):
            driver.movel([0.5, 0.2, 0.3, 0.0, 0.0, 0.0])
