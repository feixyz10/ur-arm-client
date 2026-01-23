import pytest
from pyurrr import URRDError, ConnectionError, OperationError, IKError


def test_exception_hierarchy():
    """Test that all custom exceptions inherit from URRDError."""
    assert issubclass(ConnectionError, URRDError)
    assert issubclass(OperationError, URRDError)
    assert issubclass(IKError, OperationError)


def test_exception_can_be_raised_and_caught():
    """Test that exceptions can be raised and caught properly."""
    with pytest.raises(URRDError):
        raise URRDError("Base error")

    with pytest.raises(ConnectionError):
        raise ConnectionError("Connection failed")

    with pytest.raises(OperationError):
        raise OperationError("Operation failed")

    with pytest.raises(IKError):
        raise IKError("No IK solution")

    # Test that IKError can be caught as OperationError or URRDError
    try:
        raise IKError("No solution")
    except OperationError:
        pass
    except URRDError:
        pytest.fail("Should be caught by OperationError first")


def test_exception_message():
    """Test that exception messages are preserved."""
    exc = IKError("No IK solution for given pose")
    assert str(exc) == "No IK solution for given pose"
    assert "No IK solution" in str(exc)
