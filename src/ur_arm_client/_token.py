"""Internal handle for one asynchronous motion."""

from dataclasses import dataclass


@dataclass(eq=False, slots=True)
class _MotionToken:
    session: object
    baseline_id: int
    expected_id: int
    done: bool = False
    error: str | None = None
