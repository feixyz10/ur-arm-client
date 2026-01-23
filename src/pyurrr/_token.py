"""Motion token for tracking non-blocking movements."""

from dataclasses import dataclass
from typing import List, Literal


@dataclass(frozen=True)
class MotionToken:
    """Token for tracking and waiting on non-blocking movements."""

    movement_type: Literal["j", "l"]  # Joint or linear movement
    target: List[float]  # Target pose (6 elements)
    velocity: float  # Velocity
    acceleration: float  # Acceleration
    blocking: bool = True  # Whether movement is blocking
