"""Read the controller version without subscribing to RTDE data."""

from collections.abc import Callable
from socket import create_connection
from struct import Struct

from ur_arm_client._exceptions import ConnectionError

_HEADER = Struct("!HB")
_VERSION = Struct("!IIII")
_NEGOTIATE_PROTOCOL = ord("V")
_GET_CONTROLLER_VERSION = ord("v")


def _request(
    send: Callable[[bytes], None],
    read: Callable[[int], bytes],
    command: int,
    payload: bytes,
    reply_size: int,
) -> bytes:
    send(_HEADER.pack(_HEADER.size + len(payload), command) + payload)
    header = read(_HEADER.size)
    if len(header) != _HEADER.size or _HEADER.unpack(header) != (
        _HEADER.size + reply_size,
        command,
    ):
        raise ConnectionError("Unexpected RTDE reply header")
    reply = read(reply_size)
    if len(reply) != reply_size:
        raise ConnectionError("Incomplete RTDE reply")
    return reply


def _read_firmware(host: str) -> tuple[int, int, int, int]:
    """Negotiate RTDE v2 and query URControl on a short-lived socket."""
    try:
        with create_connection((host, 30004), timeout=2.0) as connection:
            with connection.makefile("rb") as reader:
                accepted = _request(
                    connection.sendall, reader.read, _NEGOTIATE_PROTOCOL, b"\x00\x02", 1
                )
                if accepted != b"\x01":
                    raise ConnectionError("Controller did not accept RTDE protocol v2")
                reply = _request(
                    connection.sendall,
                    reader.read,
                    _GET_CONTROLLER_VERSION,
                    b"",
                    _VERSION.size,
                )
    except (ConnectionError, OSError) as error:
        raise ConnectionError(
            f"Could not determine firmware on {host}: {error}"
        ) from error
    major, minor, bugfix, build = _VERSION.unpack(reply)
    return major, minor, bugfix, build
