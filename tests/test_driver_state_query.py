"""Tests for state query properties (jpos, jvel, xpose)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError


def test_jpos_returns_joint_positions():
    """Test that jpos property returns current joint positions."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        jpos = driver.jpos

        assert jpos == [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        mock_recv_instance.getActualQ.assert_called_once()


def test_jvel_returns_joint_velocities():
    """Test that jvel property returns current joint velocities."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQd.return_value = [0.1, 0.0, 0.2, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        jvel = driver.jvel

        assert jvel == [0.1, 0.0, 0.2, 0.0, 0.0, 0.0]
        mock_recv_instance.getActualQd.assert_called_once()


def test_xpose_returns_tcp_pose():
    """Test that xpose property returns current TCP pose."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.getActualTCPPose.return_value = [
            0.5,
            0.2,
            0.3,
            0.0,
            0.0,
            0.0,
        ]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        xpose = driver.xpose

        assert xpose == [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        mock_recv_instance.getActualTCPPose.assert_called_once()


def test_jpos_requires_connection():
    """Test that jpos raises ConnectionError when not connected."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Not connected to robot"):
            _ = driver.jpos


def test_jvel_requires_connection():
    """Test that jvel raises ConnectionError when not connected."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Not connected to robot"):
            _ = driver.jvel


def test_xpose_requires_connection():
    """Test that xpose raises ConnectionError when not connected."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Not connected to robot"):
            _ = driver.xpose


def test_jpos_works_in_read_only_mode():
    """Test that jpos works in read-only mode."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        jpos = driver.jpos

        assert jpos == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_jvel_works_in_read_only_mode():
    """Test that jvel works in read-only mode."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQd.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        jvel = driver.jvel

        assert jvel == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_xpose_works_in_read_only_mode():
    """Test that xpose works in read-only mode."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.getActualTCPPose.return_value = [
            0.5,
            0.0,
            0.2,
            0.0,
            0.0,
            0.0,
        ]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        xpose = driver.xpose

        assert xpose == [0.5, 0.0, 0.2, 0.0, 0.0, 0.0]
