import pytest
from pyurrr import MotionToken


def test_motion_token_creation():
    """Test that MotionToken can be created with basic attributes."""
    token = MotionToken(
        movement_type="j",
        target=[0, -1.57, 1.57, 0, 0, 0],
        velocity=1.05,
        acceleration=1.4,
    )

    assert token.movement_type == "j"
    assert token.target == [0, -1.57, 1.57, 0, 0, 0]
    assert token.velocity == 1.05
    assert token.acceleration == 1.4


def test_motion_token_with_linear_type():
    """Test MotionToken with linear movement type."""
    token = MotionToken(
        movement_type="l",
        target=[0.5, 0.2, 0.3, 0, 0, 3.14],
        velocity=0.25,
        acceleration=1.2,
    )

    assert token.movement_type == "l"
    assert len(token.target) == 6


def test_motion_token_with_optional_fields():
    """Test MotionToken with optional fields."""
    token = MotionToken(
        movement_type="j",
        target=[0, 0, 0, 0, 0, 0],
        velocity=1.0,
        acceleration=1.0,
        blocking=True,
    )

    assert token.blocking is True


def test_motion_token_equality():
    """Test that two MotionTokens with same values are equal."""
    token1 = MotionToken("j", [0, 0, 0, 0, 0, 0], 1.0, 1.0)
    token2 = MotionToken("j", [0, 0, 0, 0, 0, 0], 1.0, 1.0)

    assert token1 == token2


def test_motion_token_inequality():
    """Test that MotionTokens with different values are not equal."""
    token1 = MotionToken("j", [0, 0, 0, 0, 0, 0], 1.0, 1.0)
    token2 = MotionToken("l", [0, 0, 0, 0, 0, 0], 1.0, 1.0)

    assert token1 != token2
