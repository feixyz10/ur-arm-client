"""Client for Universal Robots arms over RTDE."""

import math
import time
from collections.abc import Sequence
from types import TracebackType

from rtde_control import RTDEControlInterface as RTDEController
from rtde_receive import RTDEReceiveInterface as RTDEReceiver

from ur_arm_client._exceptions import ConnectionError, OperationError
from ur_arm_client._firmware import _read_firmware
from ur_arm_client._token import _MotionToken
from ur_arm_client._values import JCurr, JPos, JTemp, JVec, JVel, Pose, Twist, Wrench


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
        home_pose: Sequence[float] | None = None,
    ) -> None:
        self.ip = ip
        self.lazy = lazy
        self.read_only = read_only
        self.frequency = frequency
        self.home_pose = home_pose

        # Internal state
        self._controller: RTDEController | None = None
        self._receiver: RTDEReceiver | None = None
        self._connected_ip: str | None = None
        self._firmware_version: tuple[int, int, int, int] | None = None
        self._session = object()
        self._pending: _MotionToken | None = None

        # Connect immediately if not lazy
        if not lazy:
            self.connect()

    @property
    def is_connected(self) -> bool:
        """Check whether the client is connected to the robot."""
        return (
            self._receiver is not None
            and self._receiver.isConnected()
            and (
                self.read_only
                or (self._controller is not None and self._controller.isConnected())
            )
        )

    @property
    def joint_positions(self) -> JPos:
        """Get current joint positions."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return JPos.from_sequence(self._receiver.getActualQ())

    @property
    def joint_velocities(self) -> JVel:
        """Get current joint velocities."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return JVel.from_sequence(self._receiver.getActualQd())

    @property
    def tcp_pose(self) -> Pose:
        """Get current TCP pose."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return Pose.from_sequence(self._receiver.getActualTCPPose())

    @property
    def joint_currents(self) -> JCurr:
        """Get current joint currents."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return JCurr.from_sequence(self._receiver.getActualCurrent())

    @property
    def joint_temperatures(self) -> JTemp:
        """Get current joint temperatures."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return JTemp.from_sequence(self._receiver.getJointTemperatures())

    @property
    def tcp_velocity(self) -> Twist:
        """Get current TCP velocity."""
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return Twist.from_sequence(self._receiver.getActualTCPSpeed())

    @property
    def tcp_wrench(self) -> Wrench:
        """Return payload-compensated RTDE `actual_TCP_force` unchanged.

        Components use robot-base axes, not tool-local axes. The controller
        determines the moment reference point (normally the tool flange on
        e-Series and the TCP on CB3); the client does not shift it.
        """
        if not self.is_connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return Wrench.from_sequence(self._receiver.getActualTCPForce())

    @property
    def is_moving(self) -> bool:
        """Check if robot is currently moving."""
        return any(abs(v) > 0.0001 for v in self.joint_velocities)

    def fk(self, jpos: Sequence[float]) -> Pose:
        """Return the TCP pose for a JPos value or six joint angles."""
        controller = self._require_control("compute forward kinematics")
        self._validate_target_kind("j", jpos)
        if len(jpos) != 6 or not all(math.isfinite(value) for value in jpos):
            raise OperationError("fk requires 6 finite joint positions")
        return Pose.from_sequence(controller.getForwardKinematics(list(jpos)))

    def ik_check(
        self, pose: Sequence[float], *, qnear: Sequence[float] | None = None
    ) -> bool:
        """Check for a kinematic solution without moving the robot.

        Accepts a Pose or six TCP pose values. The optional qnear is a
        JPos value or six reference joint angles.
        """
        controller = self._require_control("check inverse kinematics")
        target, reference = self._ik_arguments(pose, qnear)
        return self._check_ik_solution(controller, target, reference)

    def ik(
        self, pose: Sequence[float], *, qnear: Sequence[float] | None = None
    ) -> JPos | None:
        """Solve for a joint configuration without moving the robot.

        Accepts a Pose or six TCP pose values. qnear selects the solution
        nearest a JPos value or six reference joint angles. If
        omitted, the controller uses the current joint configuration.
        Returns None only when the controller confirms no solution.
        """
        controller = self._require_control("compute inverse kinematics")
        target, reference = self._ik_arguments(pose, qnear)
        joints = controller.getInverseKinematics(target, reference)
        if len(joints) == 6 and all(math.isfinite(value) for value in joints):
            return JPos.from_sequence(joints)
        if joints:
            raise OperationError("Controller returned invalid inverse kinematics data")

        # The native solver also returns an empty vector on command failure.
        if not self.is_connected:
            raise ConnectionError("Connection lost while computing inverse kinematics")
        if not controller.isProgramRunning():
            raise OperationError(
                "RTDE control script stopped during inverse kinematics; "
                "no solution could be confirmed"
            )
        if self._check_ik_solution(controller, target, reference):
            raise OperationError(
                "Inverse kinematics failed despite a solution existing"
            )
        return None

    def _check_ik_solution(
        self, controller: RTDEController, target: list[float], reference: list[float]
    ) -> bool:
        if self._firmware_version is None:
            if self._connected_ip is None:
                raise ConnectionError("Not connected to robot")
            self._firmware_version = _read_firmware(self._connected_ip)
        major, minor, bugfix, _ = self._firmware_version
        # Match the $5.10|3.15 gate in the ur-rtde control script.
        if (major, minor) < (5, 10) and not (major == 3 and minor >= 15):
            raise OperationError(
                f"IK solution checks are not supported on controller firmware "
                f"{major}.{minor}.{bugfix}; requires PolyScope 3.15+ on CB3 "
                "or 5.10+ on later controllers"
            )
        return controller.getInverseKinematicsHasSolution(target, reference)

    @classmethod
    def _ik_arguments(
        cls, pose: Sequence[float], qnear: Sequence[float] | None
    ) -> tuple[list[float], list[float]]:
        cls._validate_target_kind("l", pose)
        target = list(pose)
        if len(target) != 6 or not all(math.isfinite(value) for value in target):
            raise OperationError("IK pose requires 6 finite pose values")
        if qnear is None:
            return target, []
        cls._validate_target_kind("j", qnear)
        reference = list(qnear)
        if len(reference) != 6 or not all(math.isfinite(value) for value in reference):
            raise OperationError("IK qnear requires 6 finite joint positions")
        return target, reference

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

    @staticmethod
    def _validate_target_kind(kind: str, target: Sequence[float]) -> None:
        if isinstance(target, (JVec, Pose, Twist, Wrench)):
            expected = JPos if kind == "j" else Pose
            if not isinstance(target, expected):
                raise OperationError(
                    f"Expected {expected.__name__}, got {type(target).__name__}"
                )

    def _move(
        self,
        kind: str,
        target: Sequence[float],
        velocity: float,
        acceleration: float,
        blocking: bool,
    ) -> _MotionToken | None:
        controller = self._require_control()
        self._validate_target_kind(kind, target)
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
        if not move(list(target), velocity, acceleration, not blocking):
            raise OperationError(f"move{kind} failed")
        self._pending = token
        return token

    def movej(
        self,
        target: Sequence[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ) -> _MotionToken | None:
        """Move to joint positions in radians."""
        return self._move("j", target, velocity, acceleration, blocking)

    def movel(
        self,
        target: Sequence[float],
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

        host = self.ip
        controller: RTDEController | None = None
        receiver: RTDEReceiver | None = None
        try:
            if not self.read_only:
                controller = RTDEController(host)
                if not controller.isConnected():
                    raise ConnectionError("RTDE control did not connect")
            receiver = RTDEReceiver(host, frequency=self.frequency)
            if not receiver.isConnected():
                raise ConnectionError("RTDE receiver did not connect")
        except Exception as error:
            cleanup_errors = self._close_interfaces(controller, receiver)
            detail = (
                f"; cleanup also failed: {cleanup_errors[0]}" if cleanup_errors else ""
            )
            raise ConnectionError(
                f"Failed to connect to robot at {host}: {error}{detail}"
            ) from error
        self._controller, self._receiver = controller, receiver
        self._connected_ip = host

    def disconnect(self) -> None:
        """Stop the control script and attempt to close both RTDE connections."""
        controller, receiver = self._controller, self._receiver
        self._controller, self._receiver = None, None
        self._connected_ip = None
        self._firmware_version = None
        self._invalidate_motion("The robot was disconnected")
        self._session = object()
        errors = self._close_interfaces(controller, receiver)
        if errors:
            details = "; ".join(str(error) for error in errors)
            raise ConnectionError(
                f"Failed to disconnect from robot: {details}"
            ) from errors[0]

    def set_home(self, pose: Sequence[float] | None) -> None:
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
        delta: Sequence[float],
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
        current = self.joint_positions
        target = [c + d for c, d in zip(current, delta, strict=True)]
        return self.movej(target, velocity, acceleration, blocking)

    def movel_relative(
        self,
        delta: Sequence[float],
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
        target = controller.poseTrans(list(self.tcp_pose), list(delta))
        return self.movel(target, velocity, acceleration, blocking)

    @staticmethod
    def _validate_delta(delta: Sequence[float]) -> None:
        if isinstance(delta, (JVec, Pose, Twist, Wrench)):
            raise OperationError(
                "Relative motion requires a sequence of offsets, not a telemetry value"
            )
        if len(delta) != 6 or not all(math.isfinite(value) for value in delta):
            raise OperationError("Relative motion requires 6 finite delta values")

    def move_path(
        self,
        waypoints: Sequence[Sequence[float]],
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
        for point in waypoints:
            self._validate_target_kind(path_type, point)
        if len(waypoints) == 0 or any(
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
