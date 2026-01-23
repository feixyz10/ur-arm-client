import sys
import pytest
from unittest.mock import Mock, patch, call, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver


def test_context_manager_connects_on_enter():
    """Test that entering context manager connects to robot."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        with URDriver(ip="192.168.1.10", lazy=True) as driver:
            assert driver.is_connected is True
            assert driver is not None


def test_context_manager_disconnects_on_exit():
    """Test that exiting context manager disconnects from robot."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        with URDriver(ip="192.168.1.10", lazy=True) as driver:
            pass

        # After exiting, should be disconnected
        mock_ctrl_instance.stopScript.assert_called_once()
        mock_ctrl_instance.disconnect.assert_called_once()
        mock_recv_instance.disconnect.assert_called_once()


def test_context_manager_disconnects_on_exception():
    """Test that context manager disconnects even if exception occurs."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        with pytest.raises(ValueError):
            with URDriver(ip="192.168.1.10", lazy=True) as driver:
                raise ValueError("Test error")

        # Should still disconnect
        mock_ctrl_instance.stopScript.assert_called_once()
        mock_ctrl_instance.disconnect.assert_called_once()


def test_context_manager_returns_self():
    """Test that context manager __enter__ returns self."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)

        with driver as d:
            assert d is driver
