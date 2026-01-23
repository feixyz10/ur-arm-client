"""pyurrr - Easy-to-use UR robot driver package.

This package provides a simple, pythonic interface for controlling
Universal Robots arms by wrapping the ur_rtde library.

Example:
    >>> from pyurrr import URDriver
    >>> with URDriver("192.168.1.10") as robot:
    ...     robot.movej([0, -1.57, 1.57, 0, 0, 0])
    ...     print(robot.jpos)
"""

from pyurrr._driver import URDriver
from pyurrr._exceptions import URRDError, ConnectionError, OperationError, IKError
from pyurrr._token import MotionToken

__all__ = [
    "URDriver",
    "URRDError",
    "ConnectionError",
    "OperationError",
    "IKError",
    "MotionToken",
]

__version__ = "0.1.0"
