"""Tests for movej (joint space movement)."""

import pytest
from unittest.mock import Mock, patch

from ur_arm_client import URArmClient, ConnectionError, OperationError


def test_movej_blocking_returns_none():
    """Test that blocking movej returns None on success."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.movej([0.0, -1.57, 1.57, 0.0, 0.0, 0.0], blocking=True)

        assert result is None
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.0, -1.57, 1.57, 0.0, 0.0, 0.0], 0.25, 1.0, False
        )


def test_movej_non_blocking_returns_token():
    """Test that non-blocking movej returns a motion token."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        status = mock_ctrl_instance.getAsyncOperationProgressEx.return_value
        status.isAsyncOperationRunning.return_value = False
        status.operationId.return_value = 0
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        result = driver.movej([0.0, -1.57, 1.57, 0.0, 0.0, 0.0], blocking=False)

        assert result is not None
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.0, -1.57, 1.57, 0.0, 0.0, 0.0], 0.25, 1.0, True
        )


def test_movej_custom_parameters():
    """Test movej with custom velocity and acceleration."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        driver.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity=0.5, acceleration=0.8)

        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5, 0.8, False
        )


def test_movej_requires_6_joint_positions():
    """Test that movej raises error for incorrect number of joints."""
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

        with pytest.raises(
            OperationError, match="movej requires 6 joint positions, got 5"
        ):
            driver.movej([0.0, 0.0, 0.0, 0.0, 0.0])

        with pytest.raises(
            OperationError, match="movej requires 6 joint positions, got 7"
        ):
            driver.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_movej_requires_connection():
    """Test that movej raises ConnectionError when not connected."""
    with (
        patch("ur_arm_client._client.RTDEController"),
        patch("ur_arm_client._client.RTDEReceiver"),
    ):
        driver = URArmClient(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Not connected to robot"):
            driver.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_movej_raises_on_failure():
    """Test that movej raises OperationError when controller fails."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = False
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        with pytest.raises(OperationError, match="movej failed"):
            driver.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_movej_raises_in_read_only_mode():
    """Test that movej raises error in read-only mode."""
    with (
        patch("ur_arm_client._client.RTDEController"),
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        with pytest.raises(OperationError, match="Cannot move in read-only mode"):
            driver.movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
