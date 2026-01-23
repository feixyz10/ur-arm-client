"""Custom exceptions for pyurrr package."""


class URRDError(Exception):
    """Base exception for all pyurrr errors."""

    pass


class ConnectionError(URRDError):
    """Exception raised for connection/disconnection failures."""

    pass


class OperationError(URRDError):
    """Exception raised for movement/operation failures."""

    pass


class IKError(OperationError):
    """Exception raised when inverse kinematics has no solution."""

    pass
