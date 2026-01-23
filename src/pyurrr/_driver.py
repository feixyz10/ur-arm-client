"""UR robot driver implementation."""

import time
from rtde_control import RTDEControlInterface as RTDEController
from rtde_receive import RTDEReceiveInterface as RTDEReceiver
from typing import List, Optional

from pyurrr._exceptions import ConnectionError, OperationError, IKError
from pyurrr._token import MotionToken


class URDriver:
    """UR robot driver.

    Wraps ur_rtde library with a simple, pythonic interface.

    Args:
        ip: Robot IP address
        read_only: If True, only connect receiver (no control)
        lazy: If True, don't connect until first use or with block
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
    ):
        self.ip = ip
        self.lazy = lazy
        self.read_only = read_only
        self.frequency = frequency
        self.home_pose = home_pose

        # Internal state
        self._controller = None
        self._receiver = None
        self._connected = False

        # Connect immediately if not lazy
        if not lazy:
            self.connect()

    @property
    def is_connected(self) -> bool:
        """Check if driver is connected to robot."""
        return self._connected

    @property
    def jpos(self) -> List[float]:
        """Get current joint positions."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualQ()

    @property
    def jvel(self) -> List[float]:
        """Get current joint velocities."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualQd()

    @property
    def xpose(self) -> List[float]:
        """Get current TCP pose."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPPose()

    @property
    def jcurrent(self) -> List[float]:
        """Get current joint currents."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualCurrent()

    @property
    def jtemp(self) -> List[float]:
        """Get current joint temperatures."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getJointTemperatures()

    @property
    def xvel(self) -> List[float]:
        """Get current TCP velocity."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPSpeed()

    @property
    def xforce(self) -> List[float]:
        """Get current TCP force."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPForce()

    @property
    def is_moving(self) -> bool:
        """Check if robot is currently moving."""
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        velocities = self.jvel
        return any(abs(v) > 0.0001 for v in velocities)

    def fk(self, jpos: List[float]) -> List[float]:
        """Forward kinematics: joint positions -> TCP pose."""
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")
        return self._controller.getForwardKinematics(jpos)

    def movej(
        self,
        target: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ):
        if self.read_only:
            raise OperationError("Cannot move in read-only mode")
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")
        if len(target) != 6:
            raise OperationError(f"movej requires 6 joint positions, got {len(target)}")

        success = self._controller.moveJ(target, velocity, acceleration, not blocking)

        if not success:
            raise OperationError("movej failed")

        if not blocking:
            return MotionToken(
                movement_type="j",
                target=target,
                velocity=velocity,
                acceleration=acceleration,
                blocking=False,
            )
        return None

    def movel(
        self,
        target: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ):
        if self.read_only:
            raise OperationError("Cannot move in read-only mode")
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")
        if len(target) != 6:
            raise OperationError(f"movel requires 6 pose values, got {len(target)}")

        success = self._controller.moveL(target, velocity, acceleration, not blocking)

        if not success:
            raise OperationError("movel failed")

        if not blocking:
            return MotionToken(
                movement_type="l",
                target=target,
                velocity=velocity,
                acceleration=acceleration,
                blocking=False,
            )
        return None

    def connect(self) -> None:
        """Establish connection to robot.

        Raises:
            ConnectionError: If connection fails
        """
        if self._connected:
            return

        try:
            if not self.read_only:
                self._controller = RTDEController(self.ip)

            self._receiver = RTDEReceiver(self.ip, frequency=self.frequency)
            self._connected = True
        except Exception as e:
            raise ConnectionError(
                f"Failed to connect to robot at {self.ip}: {e}"
            ) from e

    def disconnect(self) -> None:
        """Clean disconnect - stop motions and close connections."""
        if not self._connected:
            return

        try:
            if self._controller is not None:
                self._controller.stopScript()
                self._controller.disconnect()

            if self._receiver is not None:
                self._receiver.disconnect()
        finally:
            self._connected = False
            self._controller = None
            self._receiver = None

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
    ) -> Optional[MotionToken]:
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
    ):
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
        current = self.jpos
        target = [c + d for c, d in zip(current, delta)]
        return self.movej(target, velocity, acceleration, blocking)

    def movel_relative(
        self,
        delta: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blocking: bool = True,
    ):
        """Move TCP relative to current pose.

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
        current = self.xpose
        target = [c + d for c, d in zip(current, delta)]
        return self.movel(target, velocity, acceleration, blocking)

    def move_path(
        self,
        waypoints: List[List[float]],
        path_type: str = "j",
        velocity: float = 0.25,
        acceleration: float = 1.0,
        blend_radius: float = 0.0,
    ) -> Optional[MotionToken]:
        """Execute a path through multiple waypoints.

        Args:
            waypoints: List of target positions (each is 6 values)
            path_type: 'j' for joint space, 'l' for Cartesian space
            velocity: Motion velocity (rad/s or m/s depending on path_type)
            acceleration: Motion acceleration (rad/s² or m/s²)
            blend_radius: Blend radius between waypoints (meters)

        Returns:
            None (path movements are always blocking)

        Raises:
            ValueError: If path_type is not 'j' or 'l'
            OperationError: If in read-only mode or motion fails
            ConnectionError: If not connected to robot
        """
        if path_type not in ("j", "l"):
            raise ValueError(f"path_type must be 'j' or 'l', got '{path_type}'")
        if self.read_only:
            raise OperationError("Cannot move in read-only mode")
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        if path_type == "l":
            move_fn = self._controller.moveL
        else:
            move_fn = self._controller.moveJ

        success = move_fn(waypoints, velocity, acceleration, False)
        if not success:
            raise OperationError(f"move_path failed (path_type={path_type})")
        return None

    def stopj(self, acceleration: float = 2.0) -> None:
        """Stop joint motion with deceleration.

        Args:
            acceleration: Deceleration (rad/s²)

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")
        self._controller.stopJ(acceleration)

    def stopl(self, acceleration: float = 10.0) -> None:
        """Stop linear motion with deceleration.

        Args:
            acceleration: Deceleration (m/s²)

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")
        self._controller.stopL(acceleration)

    def wait_done(self, token: MotionToken, timeout: float = 60.0) -> None:
        """Wait for a specific movement to complete.

        Args:
            token: MotionToken from a non-blocking movement
            timeout: Maximum time to wait in seconds (default: 60)

        Raises:
            ConnectionError: If not connected to robot
            TimeoutError: If movement doesn't complete within timeout
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")

        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.is_moving:
                return
            time.sleep(0.01)

        raise TimeoutError(f"Movement did not complete within {timeout} seconds")

    def wait_all_done(self, timeout: float = 60.0) -> None:
        """Wait for all queued movements to complete.

        Args:
            timeout: Maximum time to wait in seconds (default: 60)

        Raises:
            ConnectionError: If not connected to robot
            TimeoutError: If movements don't complete within timeout
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")

        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.is_moving:
                return
            time.sleep(0.01)

        raise TimeoutError(f"Movements did not complete within {timeout} seconds")

    def __enter__(self):
        """Enter context manager and connect if not already connected.

        Returns:
            self: The URDriver instance
        """
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit context manager and disconnect.

        Args:
            exc_type: Exception type if an exception was raised
            exc_val: Exception value if an exception was raised
            exc_tb: Exception traceback if an exception was raised

        Returns:
            None: Exceptions are not suppressed
        """
        self.disconnect()
        return False  # Don't suppress exceptions
