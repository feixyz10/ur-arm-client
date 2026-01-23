import pytest
from pyurrr import URDriver


def test_constructor_with_required_params():
    """Test URDriver constructor with required parameters."""
    driver = URDriver(ip="192.168.1.10")

    assert driver.ip == "192.168.1.10"
    assert driver.read_only is False
    assert driver.lazy is True
    assert driver.frequency == 125


def test_constructor_with_all_params():
    """Test URDriver constructor with all parameters."""
    driver = URDriver(
        ip="192.168.1.10",
        read_only=True,
        lazy=False,
        frequency=250,
        home_pose=[0, -1.57, 1.57, 0, 0, 0],
    )

    assert driver.ip == "192.168.1.10"
    assert driver.read_only is True
    assert driver.lazy is False
    assert driver.frequency == 250
    assert driver.home_pose == [0, -1.57, 1.57, 0, 0, 0]


def test_constructor_default_home_pose():
    """Test that home_pose defaults to None (zero position)."""
    driver = URDriver(ip="192.168.1.10")

    assert driver.home_pose is None


def test_constructor_lazy_by_default():
    """Test that lazy connection is enabled by default."""
    driver = URDriver(ip="192.168.1.10")

    assert driver.lazy is True
    assert driver.is_connected is False  # Should not connect immediately


def test_constructor_lazy_does_not_connect():
    """Test that lazy mode does not connect immediately."""
    # Note: Connection will be mocked in integration tests
    driver = URDriver(ip="192.168.1.10", lazy=True)

    assert driver.is_connected is False
