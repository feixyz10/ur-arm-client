"""Tests for token-based wait methods (wait_done, wait_all_done)."""

import sys
import pytest
import time
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError, MotionToken


def test_wait_done_returns_when_not_moving():
    """Test that wait_done returns immediately when robot is not moving."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        # Robot is stationary
        mock_recv_instance.getActualQd.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        token = MotionToken(
            movement_type="j",
            target=[0, 0, 0, 0, 0, 0],
            velocity=1.0,
            acceleration=1.0,
            blocking=False,
        )

        # Should return immediately
        driver.wait_done(token)


def test_wait_done_waits_for_motion_to_complete():
    """Test that wait_done waits until robot stops moving."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        token = MotionToken(
            movement_type="j",
            target=[0, 0, 0, 0, 0, 0],
            velocity=1.0,
            acceleration=1.0,
            blocking=False,
        )

        # Simulate motion stopping after 2 checks
        call_count = [0]

        def mock_get_velocity():
            call_count[0] += 1
            if call_count[0] < 3:
                return [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        mock_recv_instance.getActualQd.side_effect = mock_get_velocity

        driver.wait_done(token)

        # Should have checked velocity at least a few times
        assert call_count[0] >= 3


def test_wait_done_timeout():
    """Test that wait_done raises TimeoutError when motion doesn't complete."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        # Robot never stops moving
        mock_recv_instance.getActualQd.return_value = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        token = MotionToken(
            movement_type="j",
            target=[0, 0, 0, 0, 0, 0],
            velocity=1.0,
            acceleration=1.0,
            blocking=False,
        )

        # Should timeout after short duration
        with pytest.raises(TimeoutError, match="did not complete"):
            driver.wait_done(token, timeout=0.1)


def test_wait_all_done_returns_when_not_moving():
    """Test that wait_all_done returns immediately when robot is not moving."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        # Robot is stationary
        mock_recv_instance.getActualQd.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        # Should return immediately
        driver.wait_all_done()


def test_wait_all_done_waits_for_motion_to_complete():
    """Test that wait_all_done waits until robot stops moving."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        # Simulate motion stopping after 2 checks
        call_count = [0]

        def mock_get_velocity():
            call_count[0] += 1
            if call_count[0] < 3:
                return [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        mock_recv_instance.getActualQd.side_effect = mock_get_velocity

        driver.wait_all_done()

        # Should have checked velocity at least a few times
        assert call_count[0] >= 3


def test_wait_all_done_timeout():
    """Test that wait_all_done raises TimeoutError when motion doesn't complete."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        # Robot never stops moving
        mock_recv_instance.getActualQd.return_value = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        # Should timeout after short duration
        with pytest.raises(TimeoutError, match="did not complete"):
            driver.wait_all_done(timeout=0.1)


def test_wait_done_requires_connection():
    """Test that wait_done raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    token = MotionToken(
        movement_type="j",
        target=[0, 0, 0, 0, 0, 0],
        velocity=1.0,
        acceleration=1.0,
        blocking=False,
    )

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        driver.wait_done(token)


def test_wait_all_done_requires_connection():
    """Test that wait_all_done raises ConnectionError when not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="Not connected to robot"):
        driver.wait_all_done()
