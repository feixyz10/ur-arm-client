"""Client for Universal Robots arms over RTDE."""

import math
import time
from types import TracebackType
from typing import List, Optional

from rtde_control import RTDEControlInterface as RTDEController
from rtde_receive import RTDEReceiveInterface as RTDEReceiver

from ur_arm_client._exceptions import ConnectionError, OperationError
from ur_arm_client._token import _MotionToken


class URArmClient:
    """Control and monitor a Universal Robots arm over RTDE.

    Wraps ur-rtde-py with a simple, pythonic interface.

    Args:
        ip: Robot IP address
        read_only: If True, only connect receiver (no control)
        lazy: If True, don't connect until connect() or entering a with block
        frequency: RTDE receive frequency in Hz
        home_pose: Initial home position for go_home() (None = [0, -1.57, 1.57, 0, 0, 0])
    """

    def __init__(
        self,
        ip: str,
        lazy: bool = True,
        read_only: bool = False,
        frequency: int = 125,
        home_pose: Optional[List[float]] = None,
    ) -> None:
        self.ip = ip
        self.lazy = lazy
        self.read_only = read_only
        self.frequency = frequency
        self.home_pose = home_pose

        # Internal state
        self._controller: RTDEController | None = None
        self._receiver: RTDEReceiver | None = None
        self._session = object()
        self._pending: _MotionToken | None = None

        # Connect immediately if not lazy
        if not lazy:
            self.connect()

    @property
    def is_connected(self) -> bool:
        """Check if driver is connected to robot."""
        return (
            self._receiver is not None
            and self._receiver.isConnected()
            and (
                self.read_only
                or (self._controller is not None and self._controller.isConnected())
            )
        )

    @property
    def jpos(self) -> List[float]:
        """Get current joint positions."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualQ()

    @property
    def jvel(self) -> List[float]:
        """Get current joint velocities."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualQd()

    @property
    def xpose(self) -> List[float]:
        """Get current TCP pose."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPPose()

    @property
    def jcurrent(self) -> List[float]:
        """Get current joint currents."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualCurrent()

    @property
    def jtemp(self) -> List[float]:
        """Get current joint temperatures."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getJointTemperatures()

    @property
    def xvel(self) -> List[float]:
        """Get current TCP velocity."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPSpeed()

    @property
    def xforce(self) -> List[float]:
        """Get current TCP force."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPForce()

    @property
    def is_moving(self) -> bool:
        """Check if robot is currently moving."""
        return any(abs(v) > 0.0001 for v in self.jvel)

    def fk(self, jpos: List[float]) -> List[float]:
        """Forward kinematics: joint positions -> TCP pose."""
        controller = self._require_control("compute forward kinematics")
        if len(jpos) != 6 or not all(math.isfinite(value) for value in jpos):
            raise OperationError("fk requires 6 finite joint positions")
        return controller.getForwardKinematics(jpos)

    def _require_control(self, action: str = "move") -> RTDEController:
        if self.read_only:
            raise OperationError(f"Cannot {action} in read-only mode")
        if not self.is_connected or self._controller is None:
            raise ConnectionError("Not connected to robot")
        return self._controller

    def _update_motion(self, token: _MotionToken) -> None:
        status = self._require_control().getAsyncOperationProgressEx()
        operation_id = status.operationId()
        if operation_id == token.expected_id:
            if not status.isAsyncOperationRunning():
                token.done = True
                self._pending = None
        elif operation_id != token.baseline_id:
            reason = "The controller started a different operation"
            self._invalidate_motion(reason)
            raise OperationError(reason)

    def _invalidate_motion(self, reason: str) -> None:
        if self._pending is not None:
            self._pending.error = reason
            self._pending = None

    def _require_idle(self) -> None:
        if self._pending is not None:
            self._update_motion(self._pending)
            if self._pending is not None:
                raise OperationError(
                    "A motion is already pending; wait for it or stop it first"
                )

    def _move(
        self,
        kind: str,
        target: List[float],
        velocity: float,
        acceleration: float,
        blocking: bool,
    ) -> _MotionToken | None:
        controller = self._require_control()
        label = "joint positions" if kind == "j" else "pose values"
        if len(target) != 6:
            raise OperationError(f"move{kind} requires 6 {label}, got {len(target)}")
        if not all(math.isfinite(value) for value in target):
            raise OperationError(f"move{kind} requires finite {label}")
        if not all(
            math.isfinite(value) and value > 0 for value in (velocity, acceleration)
        ):
            raise OperationError(
                "Motion velocity and acceleration must be finite and positive"
            )
        self._require_idle()
        move = controller.moveJ if kind == "j" else controller.moveL
        token = None
        if not blocking:
            status = controller.getAsyncOperationProgressEx()
            if status.isAsyncOperationRunning():
                raise OperationError(
                    "The controller already has an asynchronous operation"
                )
            baseline = status.operationId()
            # RTDE advances its seven-bit operation counter when a new move starts.
            token = _MotionToken(self._session, baseline, (baseline + 1) % 128)
        if not move(target, velocity, acceleration, not blocking):
            raise OperationError(f"move{kind} failed")
        self._pending = token
        return token

    def movej(
        self,
        target: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ) -> _MotionToken | None:
        """Move to joint positions in radians."""
        return self._move("j", target, velocity, acceleration, blocking)

    def movel(
        self,
        target: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ) -> _MotionToken | None:
        """Move linearly to a TCP pose in the robot base frame."""
        return self._move("l", target, velocity, acceleration, blocking)

    @staticmethod
    def _close_interfaces(
        controller: RTDEController | None, receiver: RTDEReceiver | None
    ) -> list[Exception]:
        errors = []
        if controller is not None:
            for close in (controller.stopScript, controller.disconnect):
                try:
                    close()
                except Exception as error:
                    errors.append(error)
        if receiver is not None:
            try:
                receiver.disconnect()
            except Exception as error:
                errors.append(error)
        return errors

    def connect(self) -> None:
        """Open the RTDE connections, or leave the client disconnected on failure."""
        if self.is_connected:
            return
        if self._controller is not None or self._receiver is not None:
            self.disconnect()

        controller: RTDEController | None = None
        receiver: RTDEReceiver | None = None
        try:
            if not self.read_only:
                controller = RTDEController(self.ip)
                if not controller.isConnected():
                    raise ConnectionError("RTDE control did not connect")
            receiver = RTDEReceiver(self.ip, frequency=self.frequency)
            if not receiver.isConnected():
                raise ConnectionError("RTDE receiver did not connect")
        except Exception as error:
            cleanup_errors = self._close_interfaces(controller, receiver)
            detail = (
                f"; cleanup also failed: {cleanup_errors[0]}" if cleanup_errors else ""
            )
            raise ConnectionError(
                f"Failed to connect to robot at {self.ip}: {error}{detail}"
            ) from error
        self._controller, self._receiver = controller, receiver

    def disconnect(self) -> None:
        """Stop the control script and attempt to close both RTDE connections."""
        controller, receiver = self._controller, self._receiver
        self._controller, self._receiver = None, None
        self._invalidate_motion("The robot was disconnected")
        self._session = object()
        errors = self._close_interfaces(controller, receiver)
        if errors:
            details = "; ".join(str(error) for error in errors)
            raise ConnectionError(
                f"Failed to disconnect from robot: {details}"
            ) from errors[0]

    def set_home(self, pose: Optional[List[float]]) -> None:
        """Set the home position for go_home().

        Args:
            pose: Home position as 6 joint values, or None to reset
        """
        self.home_pose = pose

    def go_home(
        self,
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ) -> _MotionToken | None:
        """Move robot to home position.

        Args:
            velocity: Joint velocity (rad/s)
            acceleration: Joint acceleration (rad/s²)
            blocking: Whether to wait for motion to complete

        Returns:
            None if blocking=True (default), MotionToken if blocking=False

        Raises:
            ConnectionError: If not connected to robot
        """
        home = (
            self.home_pose if self.home_pose is not None else [0, -1.57, 1.57, 0, 0, 0]
        )
        return self.movej(
            home, velocity=velocity, acceleration=acceleration, blocking=blocking
        )

    def movej_relative(
        self,
        delta: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ) -> _MotionToken | None:
        """Move joints relative to current position.

        Args:
            delta: Joint position deltas (6 values)
            velocity: Joint velocity (rad/s)
            acceleration: Joint acceleration (rad/s²)
            blocking: Whether to wait for motion to complete

        Returns:
            None if blocking=True, MotionToken if blocking=False

        Raises:
            ConnectionError: If not connected to robot
        """
        self._require_control()
        self._validate_delta(delta)
        self._require_idle()
        current = self.jpos
        target = [c + d for c, d in zip(current, delta, strict=True)]
        return self.movej(target, velocity, acceleration, blocking)

    def movel_relative(
        self,
        delta: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ) -> _MotionToken | None:
        """Move TCP by a pose offset expressed in the current tool frame.

        Args:
            delta: TCP pose deltas (6 values: x, y, z, rx, ry, rz)
            velocity: TCP velocity (m/s)
            acceleration: TCP acceleration (m/s²)
            blocking: Whether to wait for motion to complete

        Returns:
            None if blocking=True, MotionToken if blocking=False

        Raises:
            ConnectionError: If not connected to robot
        """
        controller = self._require_control()
        self._validate_delta(delta)
        self._require_idle()
        target = controller.poseTrans(self.xpose, delta)
        return self.movel(target, velocity, acceleration, blocking)

    @staticmethod
    def _validate_delta(delta: List[float]) -> None:
        if len(delta) != 6 or not all(math.isfinite(value) for value in delta):
            raise OperationError("Relative motion requires 6 finite delta values")

    def move_path(
        self,
        waypoints: List[List[float]],
        path_type: str = "j",
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blend_radius: float = 0.0,
    ) -> None:
        """Run a blocking joint or TCP path. Blend intermediate points only.

        Each waypoint has six coordinates. Joint paths use rad/s and rad/s²;
        TCP paths use m/s and m/s². Blend radius is in meters.
        """
        if path_type not in ("j", "l"):
            raise ValueError(f"path_type must be 'j' or 'l', got '{path_type}'")
        controller = self._require_control()
        if not waypoints or any(
            len(point) != 6 or not all(math.isfinite(value) for value in point)
            for point in waypoints
        ):
            raise ValueError("Path requires at least one waypoint with 6 finite values")
        if not all(
            math.isfinite(value) and value > 0 for value in (velocity, acceleration)
        ):
            raise ValueError(
                "Path velocity and acceleration must be finite and positive"
            )
        if not math.isfinite(blend_radius) or blend_radius < 0:
            raise ValueError("Blend radius must be finite and non-negative")
        self._require_idle()
        path = [
            [
                *point,
                velocity,
                acceleration,
                blend_radius if index < len(waypoints) - 1 else 0.0,
            ]
            for index, point in enumerate(waypoints)
        ]
        move = controller.moveJ if path_type == "j" else controller.moveL
        if not move(path, False):
            raise OperationError(f"move_path failed (path_type={path_type})")

    def stopj(self, acceleration: float = 2.0) -> None:
        """Stop joint motion with deceleration in rad/s²."""
        self._stop("j", acceleration)

    def stopl(self, acceleration: float = 10.0) -> None:
        """Stop linear motion with deceleration in m/s²."""
        self._stop("l", acceleration)

    def _stop(self, kind: str, acceleration: float) -> None:
        controller = self._require_control("stop motion")
        if not math.isfinite(acceleration) or acceleration <= 0:
            raise OperationError("Stop deceleration must be finite and positive")
        self._invalidate_motion(f"Motion was interrupted by stop{kind}()")
        stop = controller.stopJ if kind == "j" else controller.stopL
        stop(acceleration)

    @staticmethod
    def _validate_timeout(timeout: float) -> None:
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("Timeout must be finite and non-negative")

    def wait_done(self, token: _MotionToken, timeout: float = 60.0) -> None:
        """Wait for the RTDE operation represented by a token from this session."""
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")
        if not isinstance(token, _MotionToken) or token.session is not self._session:
            raise OperationError("Motion token does not belong to this connection")
        self._validate_timeout(timeout)
        deadline = time.monotonic() + timeout
        while not token.done:
            if token.error is not None:
                raise OperationError(token.error)
            if self._pending is not token:
                raise OperationError("Motion token is no longer pending")
            self._update_motion(token)
            if token.done:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"Movement did not complete within {timeout} seconds"
                )
            time.sleep(min(0.01, remaining))

    def wait_all_done(self, timeout: float = 60.0) -> None:
        """Wait for the client's pending motion, if any. There is no queue."""
        if not self.is_connected:
            raise ConnectionError("Not connected to robot")
        if self._pending is not None:
            self.wait_done(self._pending, timeout)
        else:
            self._validate_timeout(timeout)

    def __enter__(self) -> "URArmClient":
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> bool:
        try:
            self.disconnect()
        except ConnectionError as error:
            if exc_val is None:
                raise
            # Preserve the original exception raised inside the with block.
            if hasattr(exc_val, "add_note"):
                exc_val.add_note(str(error))
        return False
