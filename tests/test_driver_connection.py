import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError


def test_connect_lazy_mode():
    """Test that lazy mode doesn't connect until connect() is called."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        assert driver.is_connected is False
        mock_ctrl.assert_not_called()
        mock_recv.assert_not_called()


def test_connect_explicit():
    """Test explicit connect() call."""
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
        driver.connect()

        assert driver.is_connected is True
        mock_ctrl.assert_called_once_with("192.168.1.10")
        mock_recv.assert_called_once_with("192.168.1.10", frequency=125)


def test_connect_read_only_mode():
    """Test that read_only mode only connects receiver."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        assert driver.is_connected is True
        mock_ctrl.assert_not_called()
        mock_recv.assert_called_once_with("192.168.1.10", frequency=125)


def test_disconnect():
    """Test disconnect stops script and closes connections."""
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
        driver.connect()
        assert driver.is_connected is True

        driver.disconnect()

        assert driver.is_connected is False
        mock_ctrl_instance.stopScript.assert_called_once()
        mock_ctrl_instance.disconnect.assert_called_once()
        mock_recv_instance.disconnect.assert_called_once()


def test_connect_when_already_connected():
    """Test that connecting when already connected doesn't create new connections."""
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
        driver.connect()
        driver.connect()  # Connect again

        # Should still only have one call
        assert mock_ctrl.call_count == 1
        assert mock_recv.call_count == 1


def test_connection_failure():
    """Test that connection failures raise ConnectionError."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl:
        mock_ctrl.side_effect = Exception("Connection refused")

        driver = URDriver(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Connection refused"):
            driver.connect()
