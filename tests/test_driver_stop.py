"""Tests for stop methods (stopj, stopl)."""

import pytest
from unittest.mock import Mock, patch

from ur_arm_client import URArmClient, ConnectionError


def test_stopj_stops_joint_motion_with_default_acceleration():
    """Test that stopj stops joint motion with default acceleration."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.stopj()

        mock_ctrl_instance.stopJ.assert_called_once_with(2.0)


def test_stopj_stops_joint_motion_with_custom_acceleration():
    """Test that stopj stops joint motion with custom acceleration."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.stopj(acceleration=3.5)

        mock_ctrl_instance.stopJ.assert_called_once_with(3.5)


def test_stopl_stops_linear_motion_with_default_acceleration():
    """Test that stopl stops linear motion with default acceleration."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.stopl()

        mock_ctrl_instance.stopL.assert_called_once_with(10.0)


def test_stopl_stops_linear_motion_with_custom_acceleration():
    """Test that stopl stops linear motion with custom acceleration."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.stopl(acceleration=15.0)

        mock_ctrl_instance.stopL.assert_called_once_with(15.0)


def test_stopj_requires_connection():
    """Test that stopj raises ConnectionError when not connected."""
    driver = URArmClient(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        driver.stopj()


def test_stopl_requires_connection():
    """Test that stopl raises ConnectionError when not connected."""
    driver = URArmClient(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        driver.stopl()
