"""Tests for additional state properties (jcurrent, jtemp, xvel, xforce, is_moving)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError


def test_jcurrent_returns_joint_currents():
    """Test that jcurrent returns actual joint currents."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        expected_currents = [1.2, -0.8, 0.5, 0.3, -0.2, 0.1]
        mock_recv_instance.getActualCurrent.return_value = expected_currents
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.jcurrent

        assert result == expected_currents
        mock_recv_instance.getActualCurrent.assert_called_once()


def test_jtemp_returns_joint_temperatures():
    """Test that jtemp returns actual joint temperatures."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        expected_temps = [45.2, 50.1, 48.3, 42.5, 41.2, 40.8]
        mock_recv_instance.getJointTemperatures.return_value = expected_temps
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.jtemp

        assert result == expected_temps
        mock_recv_instance.getJointTemperatures.assert_called_once()


def test_xvel_returns_tcp_velocity():
    """Test that xvel returns actual TCP velocity."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        expected_velocity = [0.01, 0.02, -0.01, 0.0, 0.0, 0.0]
        mock_recv_instance.getActualTCPSpeed.return_value = expected_velocity
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.xvel

        assert result == expected_velocity
        mock_recv_instance.getActualTCPSpeed.assert_called_once()


def test_xforce_returns_tcp_force():
    """Test that xforce returns actual TCP force."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        expected_force = [10.5, -5.2, 20.1, 0.5, -0.3, 0.1]
        mock_recv_instance.getActualTCPForce.return_value = expected_force
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.xforce

        assert result == expected_force
        mock_recv_instance.getActualTCPForce.assert_called_once()


def test_is_moving_detects_motion():
    """Test that is_moving returns True when joints are moving."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        # Joint velocities indicating motion
        mock_recv_instance.getActualQd.return_value = [0.1, 0.0, -0.05, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.is_moving

        assert result is True


def test_is_moving_detects_stationary():
    """Test that is_moving returns False when joints are stationary."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        # Joint velocities all near zero
        mock_recv_instance.getActualQd.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.is_moving

        assert result is False


def test_jcurrent_requires_connection():
    """Test that jcurrent raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        _ = driver.jcurrent


def test_xvel_requires_connection():
    """Test that xvel raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        _ = driver.xvel


def test_xforce_requires_connection():
    """Test that xforce raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        _ = driver.xforce


def test_is_moving_requires_connection():
    """Test that is_moving raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        _ = driver.is_moving


def test_jtemp_requires_connection():
    """Test that jtemp raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        _ = driver.jtemp
