import pytest
from unittest.mock import Mock, patch

from ur_arm_client import URArmClient, ConnectionError


def test_connect_lazy_mode():
    """Test that lazy mode doesn't connect until connect() is called."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        driver = URArmClient(ip="192.168.1.10", lazy=True)

        assert driver.is_connected is False
        mock_ctrl.assert_not_called()
        mock_recv.assert_not_called()


def test_connect_explicit():
    """Test explicit connect() call."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        assert driver.is_connected is True
        mock_ctrl.assert_called_once_with("192.168.1.10")
        mock_recv.assert_called_once_with("192.168.1.10", frequency=125)


def test_connect_read_only_mode():
    """Test that read_only mode only connects receiver."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        assert driver.is_connected is True
        mock_ctrl.assert_not_called()
        mock_recv.assert_called_once_with("192.168.1.10", frequency=125)


def test_disconnect():
    """Test disconnect stops script and closes connections."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
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
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()
        driver.connect()  # Connect again

        # Should still only have one call
        assert mock_ctrl.call_count == 1
        assert mock_recv.call_count == 1


def test_connection_failure():
    """Test that connection failures raise ConnectionError."""
    with patch("ur_arm_client._client.RTDEController") as mock_ctrl:
        mock_ctrl.side_effect = Exception("Connection refused")

        driver = URArmClient(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Connection refused"):
            driver.connect()


def test_failed_receiver_rolls_back_control_and_allows_retry():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver") as receive,
    ):
        first_control = Mock()
        control.side_effect = [first_control, Mock()]
        receive.side_effect = [RuntimeError("receiver refused"), Mock()]
        client = URArmClient("robot")

        with pytest.raises(ConnectionError, match="receiver refused"):
            client.connect()

        assert not client.is_connected
        first_control.stopScript.assert_called_once()
        first_control.disconnect.assert_called_once()

        client.connect()
        assert client.is_connected
        client.disconnect()


def test_receiver_reporting_disconnected_is_closed():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver") as receive,
    ):
        receive.return_value.isConnected.return_value = False
        client = URArmClient("robot")

        with pytest.raises(ConnectionError, match="receiver did not connect"):
            client.connect()

        assert not client.is_connected
        control.return_value.disconnect.assert_called_once()
        receive.return_value.disconnect.assert_called_once()


def test_disconnect_closes_both_connections_even_if_stop_script_fails():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver") as receive,
    ):
        control.return_value.stopScript.side_effect = RuntimeError("stop failed")
        client = URArmClient("robot")
        client.connect()

        with pytest.raises(ConnectionError, match="stop failed"):
            client.disconnect()

        control.return_value.disconnect.assert_called_once()
        receive.return_value.disconnect.assert_called_once()
        assert not client.is_connected
        client.disconnect()


def test_connection_status_reflects_transport_loss():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
    ):
        client = URArmClient("robot")
        client.connect()
        assert client.is_connected

        control.return_value.isConnected.return_value = False
        assert not client.is_connected

        client.disconnect()
        control.return_value.disconnect.assert_called_once()


def test_cleanup_failure_does_not_mask_error_in_with_block():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver") as receive,
    ):
        control.return_value.stopScript.side_effect = RuntimeError("stop failed")
        with pytest.raises(ValueError, match="original failure"):
            with URArmClient("robot"):
                raise ValueError("original failure")
        receive.return_value.disconnect.assert_called_once()
