"""Exercise the version query against RTDE wire-format replies."""

from io import BytesIO
from struct import pack
from unittest.mock import MagicMock, call, patch

import pytest

from ur_arm_client import ConnectionError
from ur_arm_client._firmware import _read_firmware

ACCEPTED = b"\x00\x04V\x01"
VERSION = pack("!HBIIII", 19, ord("v"), 5, 10, 2, 1234)


def fake_connection(data):
    sock = MagicMock()
    sock.__enter__.return_value = sock
    sock.makefile.return_value = BytesIO(data)
    return sock


def test_version_query_does_not_configure_or_start_an_rtde_stream():
    sock = fake_connection(ACCEPTED + VERSION)
    with patch(
        "ur_arm_client._firmware.create_connection", return_value=sock
    ) as connect:
        assert _read_firmware("robot") == (5, 10, 2, 1234)

    connect.assert_called_once_with(("robot", 30004), timeout=2.0)
    assert sock.sendall.call_args_list == [
        call(b"\x00\x05V\x00\x02"),
        call(b"\x00\x03v"),
    ]
    sock.__exit__.assert_called_once()


@pytest.mark.parametrize(
    ("reply", "error"),
    [
        (b"\x00\x04V\x00", "did not accept"),
        (ACCEPTED + b"\x00\x13M", "Unexpected RTDE reply header"),
        (ACCEPTED + VERSION[:-1], "Incomplete RTDE reply"),
    ],
)
def test_invalid_firmware_replies_fail_closed(reply, error):
    sock = fake_connection(reply)
    with (
        patch("ur_arm_client._firmware.create_connection", return_value=sock),
        pytest.raises(ConnectionError, match=error),
    ):
        _read_firmware("robot")
    sock.__exit__.assert_called_once()


def test_network_errors_propagate_as_connection_errors():
    with (
        patch(
            "ur_arm_client._firmware.create_connection",
            side_effect=TimeoutError("timeout"),
        ),
        pytest.raises(ConnectionError, match="Could not determine firmware on robot"),
    ):
        _read_firmware("robot")
