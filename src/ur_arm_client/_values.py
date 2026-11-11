"""Immutable snapshots of Cartesian and joint telemetry."""

from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from typing import TypeVar, overload

_Vector3 = tuple[float, float, float]
_Vector6 = tuple[float, float, float, float, float, float]
_JOINT_NAMES = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")


def _triple(values: Sequence[float], name: str) -> _Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} requires exactly 3 values")
    x, y, z = values
    return float(x), float(y), float(z)


def _split(values: Sequence[float], name: str) -> tuple[_Vector3, _Vector3]:
    if len(values) != 6:
        raise ValueError(f"{name} requires exactly 6 values")
    a, b, c, d, e, f = values
    return (a, b, c), (d, e, f)


class _CartesianVector(Sequence[float]):
    __slots__ = ()

    def _as_tuple(self) -> _Vector6:
        raise NotImplementedError

    def __len__(self) -> int:
        return 6

    def __iter__(self) -> Iterator[float]:
        return iter(self._as_tuple())

    @overload
    def __getitem__(self, index: int) -> float: ...

    @overload
    def __getitem__(self, index: slice) -> tuple[float, ...]: ...

    def __getitem__(self, index: int | slice) -> float | tuple[float, ...]:
        return self._as_tuple()[index]


@dataclass(frozen=True, slots=True)
class Pose(_CartesianVector):
    """Position in meters and rotation vector in radians.

    The sequence order is (x, y, z, rx, ry, rz). Values read from the client
    describe the TCP relative to the robot base.
    """

    position: _Vector3
    rotvec: _Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _triple(self.position, "position"))
        object.__setattr__(self, "rotvec", _triple(self.rotvec, "rotvec"))

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "Pose":
        """Create a pose from six values in UR order."""
        return cls(*_split(values, cls.__name__))

    def _as_tuple(self) -> _Vector6:
        return (*self.position, *self.rotvec)


@dataclass(frozen=True, slots=True)
class Twist(_CartesianVector):
    """Linear velocity in m/s followed by angular velocity in rad/s."""

    linear: _Vector3
    angular: _Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "linear", _triple(self.linear, "linear"))
        object.__setattr__(self, "angular", _triple(self.angular, "angular"))

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "Twist":
        """Create a twist from three linear and three angular components."""
        return cls(*_split(values, cls.__name__))

    def _as_tuple(self) -> _Vector6:
        return (*self.linear, *self.angular)


@dataclass(frozen=True, slots=True)
class Wrench(_CartesianVector):
    """Force in newtons followed by torque in newton-meters."""

    force: _Vector3
    torque: _Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "force", _triple(self.force, "force"))
        object.__setattr__(self, "torque", _triple(self.torque, "torque"))

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> "Wrench":
        """Create a wrench from three force and three torque components."""
        return cls(*_split(values, cls.__name__))

    def _as_tuple(self) -> _Vector6:
        return (*self.force, *self.torque)


_JVecT = TypeVar("_JVecT", bound="JVec")


@dataclass(frozen=True, slots=True)
class JVec(Sequence[float]):
    """Six named joint values, ordered from base to wrist3."""

    base: float
    shoulder: float
    elbow: float
    wrist1: float
    wrist2: float
    wrist3: float

    def __post_init__(self) -> None:
        for name in _JOINT_NAMES:
            object.__setattr__(self, name, float(getattr(self, name)))

    @classmethod
    def from_sequence(cls: type[_JVecT], values: Sequence[float]) -> _JVecT:
        """Create a joint vector from six values in UR order."""
        if len(values) != 6:
            raise ValueError(f"{cls.__name__} requires exactly 6 values")
        return cls(*values)

    def _as_tuple(self) -> _Vector6:
        return (
            self.base,
            self.shoulder,
            self.elbow,
            self.wrist1,
            self.wrist2,
            self.wrist3,
        )

    def __len__(self) -> int:
        return 6

    def __iter__(self) -> Iterator[float]:
        return iter(self._as_tuple())

    @overload
    def __getitem__(self, index: int | str) -> float: ...

    @overload
    def __getitem__(self, index: slice) -> tuple[float, ...]: ...

    def __getitem__(self, index: int | slice | str) -> float | tuple[float, ...]:
        if isinstance(index, str):
            if index not in _JOINT_NAMES:
                raise KeyError(index)
            return getattr(self, index)
        return self._as_tuple()[index]


class JPos(JVec):
    """Joint angles in radians."""

    __slots__ = ()


class JVel(JVec):
    """Joint angular velocities in radians per second."""

    __slots__ = ()


class JCurr(JVec):
    """Joint currents in amperes."""

    __slots__ = ()


class JTemp(JVec):
    """Joint temperatures in degrees Celsius."""

    __slots__ = ()
