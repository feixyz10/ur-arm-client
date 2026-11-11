"""Custom exceptions for ur_arm_client package."""


class URRDError(Exception):
    """Base exception for all ur_arm_client errors."""

    pass


class ConnectionError(URRDError):
    """Exception raised for connection/disconnection failures."""

    pass


class OperationError(URRDError):
    """Exception raised for movement/operation failures."""

    pass


class IKError(OperationError):
    """Reserved inverse kinematics error; confirmed no solution returns None."""

    pass
