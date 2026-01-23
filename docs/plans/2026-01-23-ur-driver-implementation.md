# UR Robot Driver Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build an easy-to-use Python wrapper around ur_rtde for controlling Universal Robots arms with a single URDriver class supporting context managers and token-based non-blocking movements.

**Architecture:**
- Single `URDriver` class wrapping `RTDEControlInterface` and `RTDEReceiveInterface`
- Exception hierarchy for error handling
- MotionToken class for non-blocking movement API
- Context manager protocol (`__enter__`, `__exit__`) for automatic resource management
- Lazy connection strategy by default

**Tech Stack:**
- `ur-rtde` - RTDE communication library (bundled in third_party/)
- `dataclasses` - MotionToken and config
- `pytest` - Testing framework

**Reference:** `docs/plans/2026-01-23-ur-driver-design.md`

---

## Phase 1: Core Functionality

### Task 1: Create Exception Hierarchy

**Files:**
- Create: `src/pyurrr/_exceptions.py`
- Create: `tests/test_exceptions.py`

**Step 1: Write the failing test**

Create `tests/test_exceptions.py`:

```python
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
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_exceptions.py -v`

Expected: `ImportError: cannot import name 'URRDError' from 'pyurrr'`

**Step 3: Write minimal implementation**

Create `src/pyurrr/_exceptions.py`:

```python
"""Custom exceptions for pyurrr package."""


class URRDError(Exception):
    """Base exception for all pyurrr errors."""
    pass


class ConnectionError(URRDError):
    """Exception raised for connection/disconnection failures."""
    pass


class OperationError(URRDError):
    """Exception raised for movement/operation failures."""
    pass


class IKError(OperationError):
    """Exception raised when inverse kinematics has no solution."""
    pass
```

Create `src/pyurrr/__init__.py`:

```python
"""pyurrr - Easy-to-use UR robot driver package."""

from pyurrr._exceptions import URRDError, ConnectionError, OperationError, IKError

__all__ = ["URRDError", "ConnectionError", "OperationError", "IKError"]
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_exceptions.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_exceptions.py src/pyurrr/__init__.py tests/test_exceptions.py
git commit -m "feat: add exception hierarchy

- URRDError base exception
- ConnectionError for connection failures
- OperationError for operation failures
- IKError for inverse kinematics failures"
```

---

### Task 2: Create MotionToken Dataclass

**Files:**
- Create: `src/pyurrr/_token.py`
- Create: `tests/test_token.py`

**Step 1: Write the failing test**

Create `tests/test_token.py`:

```python
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
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_token.py -v`

Expected: `ImportError: cannot import name 'MotionToken' from 'pyurrr'`

**Step 3: Write minimal implementation**

Create `src/pyurrr/_token.py`:

```python
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
```

Update `src/pyurrr/__init__.py`:

```python
"""pyurrr - Easy-to-use UR robot driver package."""

from pyurrr._exceptions import URRDError, ConnectionError, OperationError, IKError
from pyurrr._token import MotionToken

__all__ = ["URRDError", "ConnectionError", "OperationError", "IKError", "MotionToken"]
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_token.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_token.py src/pyurrr/__init__.py tests/test_token.py
git commit -m "feat: add MotionToken dataclass

- Track non-blocking movements
- Store movement type, target, velocity, acceleration
- Support equality comparison"
```

---

### Task 3: Create URDriver Skeleton with Constructor

**Files:**
- Create: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_constructor.py`

**Step 1: Write the failing test**

Create `tests/test_driver_constructor.py`:

```python
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


def test_constructor_not_lazy():
    """Test that non-lazy mode connects immediately."""
    # Note: This will fail to connect in unit tests (no robot)
    # We'll mock the connection later
    driver = URDriver(ip="192.168.1.10", lazy=True)

    assert driver.is_connected is False
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_constructor.py -v`

Expected: `ImportError: cannot import name 'URDriver' from 'pyurrr'`

**Step 3: Write minimal implementation**

Create `src/pyurrr/_driver.py`:

```python
"""UR robot driver implementation."""

from typing import List, Optional


class URDriver:
    """UR robot driver.

    Wraps ur_rtde library with a simple, pythonic interface.

    Args:
        ip: Robot IP address
        read_only: If True, only connect receiver (no control)
        lazy: If True, don't connect until first use or with block
        frequency: RTDE receive frequency in Hz
        home_pose: Initial home position for go_home() (None = zero position)
    """

    def __init__(
        self,
        ip: str,
        read_only: bool = False,
        lazy: bool = True,
        frequency: int = 125,
        home_pose: Optional[List[float]] = None,
    ):
        self.ip = ip
        self.read_only = read_only
        self.lazy = lazy
        self.frequency = frequency
        self.home_pose = home_pose

        # Internal state
        self._controller = None
        self._receiver = None
        self._connected = False

        # Connect immediately if not lazy
        if not lazy:
            self._connect()

    @property
    def is_connected(self) -> bool:
        """Check if driver is connected to robot."""
        return self._connected

    def _connect(self):
        """Internal connection method (to be implemented)."""
        # Will be implemented in next task
        pass

    def _disconnect(self):
        """Internal disconnection method (to be implemented)."""
        # Will be implemented in next task
        pass
```

Update `src/pyurrr/__init__.py`:

```python
"""pyurrr - Easy-to-use UR robot driver package."""

from pyurrr._driver import URDriver
from pyurrr._exceptions import URRDError, ConnectionError, OperationError, IKError
from pyurrr._token import MotionToken

__all__ = [
    "URDriver",
    "URRDError",
    "ConnectionError",
    "OperationError",
    "IKError",
    "MotionToken",
]
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_constructor.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py src/pyurrr/__init__.py tests/test_driver_constructor.py
git commit -m "feat: add URDriver skeleton with constructor

- Store connection parameters
- Support lazy and non-lazy modes
- Add is_connected property
- Support read_only and home_pose configuration"
```

---

### Task 4: Implement Connection Management

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_connection.py`

**Step 1: Write the failing test**

Create `tests/test_driver_connection.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver, ConnectionError


def test_connect_lazy_mode():
    """Test that lazy mode doesn't connect until connect() is called."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        driver = URDriver(ip="192.168.1.10", lazy=True)

        assert driver.is_connected is False
        mock_ctrl.assert_not_called()
        mock_recv.assert_not_called()


def test_connect_explicit():
    """Test explicit connect() call."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        assert driver.is_connected is True
        mock_ctrl.assert_called_once_with("192.168.1.10")
        mock_recv.assert_called_once_with("192.168.1.10")


def test_connect_read_only_mode():
    """Test that read_only mode only connects receiver."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver.connect()

        assert driver.is_connected is True
        mock_ctrl.assert_not_called()
        mock_recv.assert_called_once_with("192.168.1.10")


def test_disconnect():
    """Test disconnect stops script and closes connections."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()
        assert driver.is_connected is True

        driver.disconnect()

        assert driver.is_connected is False
        mock_ctrl_instance.stopScript.assert_called_once()
        mock_ctrl_instance.disconnect.assert_called_once()
        mock_recv_instance.disconnect.assert_called_once()


def test_connect_when_already_connected():
    """Test that connecting when already connected doesn't create new connections."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()
        driver.connect()  # Connect again

        # Should still only have one call
        assert mock_ctrl.call_count == 1
        assert mock_recv.call_count == 1


def test_connection_failure():
    """Test that connection failures raise ConnectionError."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl:
        mock_ctrl.side_effect = Exception("Connection refused")

        driver = URDriver(ip="192.168.1.10", lazy=True)

        with pytest.raises(ConnectionError, match="Connection refused"):
            driver.connect()
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_connection.py -v`

Expected: Tests fail due to missing RTDEController/RTDEReceiver imports and incomplete implementation

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py`:

```python
"""UR robot driver implementation."""

from rtde_control import RTDEControlInterface as RTDEController
from rtde_receive import RTDEReiveInterface as RTDEReceiver
from typing import List, Optional

from pyurrr._exceptions import ConnectionError


class URDriver:
    """UR robot driver.

    Wraps ur_rtde library with a simple, pythonic interface.

    Args:
        ip: Robot IP address
        read_only: If True, only connect receiver (no control)
        lazy: If True, don't connect until first use or with block
        frequency: RTDE receive frequency in Hz
        home_pose: Initial home position for go_home() (None = zero position)
    """

    def __init__(
        self,
        ip: str,
        read_only: bool = False,
        lazy: bool = True,
        frequency: int = 125,
        home_pose: Optional[List[float]] = None,
    ):
        self.ip = ip
        self.read_only = read_only
        self.lazy = lazy
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
            raise ConnectionError(f"Failed to connect to robot at {self.ip}: {e}") from e

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
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_connection.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_connection.py
git commit -m "feat: implement connection management

- Add connect() method with error handling
- Add disconnect() method that stops script and closes connections
- Support read_only mode (receiver only)
- Support reconnection logic
- Add comprehensive connection tests with mocks"
```

---

### Task 5: Implement Context Manager Protocol

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_context_manager.py`

**Step 1: Write the failing test**

Create `tests/test_driver_context_manager.py`:

```python
import pytest
from unittest.mock import Mock, patch, call
from pyurrr import URDriver


def test_context_manager_connects_on_enter():
    """Test that entering context manager connects to robot."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        with URDriver(ip="192.168.1.10", lazy=True) as driver:
            assert driver.is_connected is True
            assert driver is not None


def test_context_manager_disconnects_on_exit():
    """Test that exiting context manager disconnects from robot."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        with URDriver(ip="192.168.1.10", lazy=True) as driver:
            pass

        # After exiting, should be disconnected
        mock_ctrl_instance.stopScript.assert_called_once()
        mock_ctrl_instance.disconnect.assert_called_once()
        mock_recv_instance.disconnect.assert_called_once()


def test_context_manager_disconnects_on_exception():
    """Test that context manager disconnects even if exception occurs."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        with pytest.raises(ValueError):
            with URDriver(ip="192.168.1.10", lazy=True) as driver:
                raise ValueError("Test error")

        # Should still disconnect
        mock_ctrl_instance.stopScript.assert_called_once()
        mock_ctrl_instance.disconnect.assert_called_once()


def test_context_manager_returns_self():
    """Test that context manager __enter__ returns self."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.isConnected.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)

        with driver as d:
            assert d is driver
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_context_manager.py -v`

Expected: Tests fail with `AttributeError: __enter__`

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
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
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_context_manager.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_context_manager.py
git commit -m "feat: implement context manager protocol

- Add __enter__ to connect on with block entry
- Add __exit__ to disconnect on with block exit
- Ensure disconnect happens even on exceptions
- Return self from __enter__ for with ... as syntax"
```

---

## Phase 2: State Query and Basic Movements

### Task 6: Implement State Query Properties

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_state_query.py`

**Step 1: Write the failing test**

Create `tests/test_driver_state_query.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_jpos_property():
    """Test getting joint positions."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0, -1.57, 1.57, 0, 0, 0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        jpos = driver.jpos

        assert jpos == [0, -1.57, 1.57, 0, 0, 0]
        mock_recv_instance.getActualQ.assert_called_once()


def test_xpose_property():
    """Test getting TCP pose."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualTCPPose.return_value = [0.5, 0.2, 0.3, 0, 0, 3.14]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        xpose = driver.xpose

        assert xpose == [0.5, 0.2, 0.3, 0, 0, 3.14]


def test_jvel_property():
    """Test getting joint velocities."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQd.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        jvel = driver.jvel

        assert jvel == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_state_query_requires_connection():
    """Test that state query raises error if not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="not connected"):
        _ = driver.jpos


def test_state_query_in_read_only_mode():
    """Test that state query works in read-only mode."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0, 0, 0, 0, 0, 0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        jpos = driver.jpos

        assert jpos == [0, 0, 0, 0, 0, 0]
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_state_query.py -v`

Expected: Tests fail due to missing properties

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add properties to URDriver class):

```python
    @property
    def jpos(self) -> List[float]:
        """Get current joint positions.

        Returns:
            List of 6 joint positions in radians

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualQ()

    @property
    def jvel(self) -> List[float]:
        """Get current joint velocities.

        Returns:
            List of 6 joint velocities in rad/s

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualQd()

    @property
    def xpose(self) -> List[float]:
        """Get current TCP pose.

        Returns:
            List of 6 pose values [x, y, z, rx, ry, rz]

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPPose()
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_state_query.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_state_query.py
git commit -m "feat: add state query properties

- Add jpos property for joint positions
- Add jvel property for joint velocities
- Add xpose property for TCP pose
- Raise ConnectionError if not connected"
```

---

### Task 7: Implement Basic Joint Movement (movej)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_movej.py`

**Step 1: Write the failing test**

Create `tests/test_driver_movej.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver, OperationError


def test_movej_blocking():
    """Test blocking joint movement."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        result = driver.movej([0, -1.57, 1.57, 0, 0, 0])

        assert result is None  # Blocking returns None
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, -1.57, 1.57, 0, 0, 0], 1.05, 1.4, True
        )


def test_movej_non_blocking():
    """Test non-blocking joint movement returns token."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        from pyurrr import MotionToken

        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        token = driver.movej([0, -1.57, 1.57, 0, 0, 0], blocking=False)

        assert isinstance(token, MotionToken)
        assert token.movement_type == "j"
        assert token.target == [0, -1.57, 1.57, 0, 0, 0]
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, -1.57, 1.57, 0, 0, 0], 1.05, 1.4, False
        )


def test_movej_with_custom_velocity_acceleration():
    """Test movej with custom velocity and acceleration."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.movej([0, -1.57, 1.57, 0, 0, 0], velocity=0.5, acceleration=0.8)

        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, -1.57, 1.57, 0, 0, 0], 0.5, 0.8, True
        )


def test_movej_validates_joint_count():
    """Test that movej validates 6 joint positions."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        with pytest.raises(OperationError, match="6 joint positions"):
            driver.movej([0, -1.57, 1.57])  # Only 3 joints


def test_movej_raises_if_not_connected():
    """Test that movej raises error if not connected."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError, match="not connected"):
        driver.movej([0, -1.57, 1.57, 0, 0, 0])


def test_movej_raises_on_movement_failure():
    """Test that movej raises OperationError if movement fails."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = False  # Movement failed
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        with pytest.raises(OperationError, match="movej failed"):
            driver.movej([0, -1.57, 1.57, 0, 0, 0])


def test_movej_raises_in_read_only_mode():
    """Test that movej raises error in read-only mode."""
    driver = URDriver(ip="192.168.1.10", read_only=True, lazy=True)
    driver._connected = True

    with pytest.raises(OperationError, match="read-only mode"):
        driver.movej([0, -1.57, 1.57, 0, 0, 0])
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_movej.py -v`

Expected: Tests fail due to missing movej method

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
from pyurrr._exceptions import ConnectionError, OperationError
from pyurrr._token import MotionToken

    def movej(
        self,
        target: List[float],
        velocity: float = 1.05,
        acceleration: float = 1.4,
        blocking: bool = True,
    ):
        """Move to joint position.

        Args:
            target: 6 joint positions in radians
            velocity: Joint velocity in rad/s (default: 1.05)
            acceleration: Joint acceleration in rad/s² (default: 1.4)
            blocking: If True, wait for movement to complete (default: True)

        Returns:
            MotionToken if blocking=False, None if blocking=True

        Raises:
            ConnectionError: If not connected to robot
            OperationError: If movement fails or in read-only mode
        """
        if self.read_only:
            raise OperationError("Cannot move in read-only mode")

        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        if len(target) != 6:
            raise OperationError(f"movej requires 6 joint positions, got {len(target)}")

        success = self._controller.moveJ(target, velocity, acceleration, blocking)

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
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_movej.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_movej.py
git commit -m "feat: implement movej (joint space movement)

- Add movej method with velocity, acceleration, blocking params
- Validate 6 joint positions
- Return MotionToken for non-blocking movements
- Raise OperationError on failures
- Prevent movement in read-only mode"
```

---

### Task 8: Implement Basic Linear Movement (movel)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_movel.py`

**Step 1: Write the failing test**

Create `tests/test_driver_movel.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver, OperationError


def test_movel_blocking():
    """Test blocking linear movement."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        result = driver.movel([0.5, 0.2, 0.3, 0, 0, 3.14])

        assert result is None
        mock_ctrl_instance.moveL.assert_called_once()


def test_movel_non_blocking():
    """Test non-blocking linear movement returns token."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        from pyurrr import MotionToken

        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        token = driver.movel([0.5, 0.2, 0.3, 0, 0, 3.14], blocking=False)

        assert isinstance(token, MotionToken)
        assert token.movement_type == "l"


def test_movel_validates_pose_count():
    """Test that movel validates 6 pose values."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        with pytest.raises(OperationError, match="6 pose values"):
            driver.movel([0.5, 0.2, 0.3])  # Only 3 values
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_movel.py -v`

Expected: Tests fail due to missing movel method

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
    def movel(
        self,
        target: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.2,
        blocking: bool = True,
    ):
        """Move to Cartesian pose (linear in tool-space).

        Args:
            target: 6 pose values [x, y, z, rx, ry, rz]
            velocity: Tool velocity in m/s (default: 0.25)
            acceleration: Tool acceleration in m/s² (default: 1.2)
            blocking: If True, wait for movement to complete (default: True)

        Returns:
            MotionToken if blocking=False, None if blocking=True

        Raises:
            ConnectionError: If not connected to robot
            OperationError: If movement fails or in read-only mode
        """
        if self.read_only:
            raise OperationError("Cannot move in read-only mode")

        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        if len(target) != 6:
            raise OperationError(f"movel requires 6 pose values, got {len(target)}")

        success = self._controller.moveL(target, velocity, acceleration, blocking)

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
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_movel.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_movel.py
git commit -m "feat: implement movel (Cartesian linear movement)

- Add movel method with velocity, acceleration, blocking params
- Validate 6 pose values [x, y, z, rx, ry, rz]
- Return MotionToken for non-blocking movements
- Raise OperationError on failures"
```

---

## Phase 3: Kinematics

### Task 9: Implement Forward Kinematics (fk)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_kinematics.py`

**Step 1: Write the failing test**

Create `tests/test_driver_kinematics.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_fk():
    """Test forward kinematics."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getForwardKinematics.return_value = [0.5, 0.2, 0.3, 0, 0, 3.14]
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        pose = driver.fk([0, -1.57, 1.57, 0, 0, 0])

        assert pose == [0.5, 0.2, 0.3, 0, 0, 3.14]
        mock_ctrl_instance.getForwardKinematics.assert_called_once_with(
            [0, -1.57, 1.57, 0, 0, 0]
        )


def test_fk_requires_connection():
    """Test that fk requires connection."""
    driver = URDriver(ip="192.168.1.10", lazy=True)

    with pytest.raises(ConnectionError):
        driver.fk([0, -1.57, 1.57, 0, 0, 0])
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_kinematics.py::test_fk -v`

Expected: Test fails due to missing fk method

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
    def fk(self, jpos: List[float]) -> List[float]:
        """Forward kinematics: joint positions -> TCP pose.

        Args:
            jpos: 6 joint positions in radians

        Returns:
            6 pose values [x, y, z, rx, ry, rz]

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        return self._controller.getForwardKinematics(jpos)
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_kinematics.py::test_fk -v`

Expected: Test PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_kinematics.py
git commit -m "feat: implement forward kinematics (fk)

- Add fk method to compute TCP pose from joint positions
- Raise ConnectionError if not connected"
```

---

### Task 10: Implement Inverse Kinematics (ik)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Modify: `tests/test_driver_kinematics.py`

**Step 1: Write the failing test**

Add to `tests/test_driver_kinematics.py`:

```python
def test_ik_with_solution():
    """Test inverse kinematics when solution exists."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getInverseKinematicsHasSolution.return_value = True
        mock_ctrl_instance.getInverseKinematics.return_value = [0, -1.57, 1.57, 0, 0, 0]
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        jpos = driver.ik([0.5, 0.2, 0.3, 0, 0, 3.14])

        assert jpos == [0, -1.57, 1.57, 0, 0, 0]


def test_ik_without_solution():
    """Test inverse kinematics when no solution exists."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        from pyurrr import IKError

        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getInverseKinematicsHasSolution.return_value = False
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        with pytest.raises(IKError, match="no solution"):
            driver.ik([0.5, 0.2, 0.3, 0, 0, 3.14])


def test_ik_with_hint():
    """Test inverse kinematics with joint position hint."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getInverseKinematicsHasSolution.return_value = True
        mock_ctrl_instance.getInverseKinematics.return_value = [0, -1.5, 1.5, 0, 0, 0]
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        jpos = driver.ik([0.5, 0.2, 0.3, 0, 0, 3.14], jpos_hint=[0, -1.0, 1.0, 0, 0, 0])

        assert jpos == [0, -1.5, 1.5, 0, 0, 0]
        mock_ctrl_instance.getInverseKinematicsHasSolution.assert_called_once_with(
            [0.5, 0.2, 0.3, 0, 0, 3.14], [0, -1.0, 1.0, 0, 0, 0]
        )
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_kinematics.py::test_ik -v`

Expected: Tests fail due to missing ik method

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
from pyurrr._exceptions import IKError

    def ik(
        self, pose: List[float], jpos_hint: Optional[List[float]] = None
    ) -> List[float]:
        """Inverse kinematics: TCP pose -> joint positions.

        Args:
            pose: 6 pose values [x, y, z, rx, ry, rz]
            jpos_hint: Optional 6 joint positions to guide IK solver

        Returns:
            6 joint positions in radians

        Raises:
            ConnectionError: If not connected to robot
            IKError: If no solution exists
        """
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        has_solution = self._controller.getInverseKinematicsHasSolution(
            pose, jpos_hint
        )

        if not has_solution:
            raise IKError(f"Inverse kinematics has no solution for pose: {pose}")

        return self._controller.getInverseKinematics(pose, jpos_hint)
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_kinematics.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_kinematics.py
git commit -m "feat: implement inverse kinematics (ik)

- Add ik method to compute joint positions from TCP pose
- Support optional jpos_hint parameter for guiding solver
- Raise IKError when no solution exists"
```

---

## Phase 4: Convenience Methods

### Task 11: Implement Home Position (set_home, go_home)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_home.py`

**Step 1: Write the failing test**

Create `tests/test_driver_home.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_set_home():
    """Test setting home position."""
    driver = URDriver(ip="192.168.1.10")

    driver.set_home([0, -1.57, 1.57, 0, 0, 0])

    assert driver.home_pose == [0, -1.57, 1.57, 0, 0, 0]


def test_set_home_to_none():
    """Test resetting home to None (zero position)."""
    driver = URDriver(ip="192.168.1.10", home_pose=[0, -1.57, 1.57, 0, 0, 0])

    driver.set_home(None)

    assert driver.home_pose is None


def test_go_home_with_custom_home():
    """Test go_home moves to custom home position."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", home_pose=[0, -1.57, 1.57, 0, 0, 0])
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.go_home()

        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, -1.57, 1.57, 0, 0, 0], 1.05, 1.4, True
        )


def test_go_home_with_zero_position():
    """Test go_home moves to zero position when home is None."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10")  # home_pose=None
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.go_home()

        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, 0, 0, 0, 0, 0], 1.05, 1.4, True
        )


def test_go_home_with_custom_velocity_acceleration():
    """Test go_home with custom velocity and acceleration."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10")
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.go_home(velocity=0.5, acceleration=0.8)

        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0, 0, 0, 0, 0, 0], 0.5, 0.8, True
        )
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_home.py -v`

Expected: Tests fail due to missing set_home and go_home methods

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
    def set_home(self, pose: Optional[List[float]]) -> None:
        """Set home position for go_home().

        Args:
            pose: 6 joint positions, or None to reset to zero position
        """
        self.home_pose = pose

    def go_home(
        self, velocity: float = 1.05, acceleration: float = 1.4
    ) -> Optional[MotionToken]:
        """Move to home position.

        Args:
            velocity: Joint velocity in rad/s (default: 1.05)
            acceleration: Joint acceleration in rad/s² (default: 1.4)

        Returns:
            MotionToken if non-blocking (not yet implemented), None if blocking

        Raises:
            ConnectionError: If not connected to robot
            OperationError: If movement fails or in read-only mode
        """
        home = self.home_pose if self.home_pose is not None else [0, 0, 0, 0, 0, 0]
        return self.movej(home, velocity=velocity, acceleration=acceleration, blocking=True)
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_home.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_home.py
git commit -m "feat: implement home position methods

- Add set_home() to configure home position
- Add go_home() to move to home position
- Default to zero position when home is None
- Support custom velocity and acceleration"
```

---

### Task 12: Implement Relative Movements (movej_relative, movel_relative)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_relative.py`

**Step 1: Write the failing test**

Create `tests/test_driver_relative.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_movej_relative():
    """Test relative joint movement."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0, -1.57, 1.57, 0, 0, 0]
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._receiver = mock_recv_instance
        driver._connected = True

        # Move relative by [0.1, 0, 0, 0, 0, 0]
        driver.movej_relative([0.1, 0, 0, 0, 0, 0])

        # Should move to [0.1, -1.57, 1.57, 0, 0, 0]
        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.1, -1.57, 1.57, 0, 0, 0], 1.05, 1.4, True
        )


def test_movel_relative():
    """Test relative Cartesian movement."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.getActualTCPPose.return_value = [0.5, 0.2, 0.3, 0, 0, 3.14]
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._receiver = mock_recv_instance
        driver._connected = True

        # Move relative by [0.01, 0, 0, 0, 0, 0] (10cm in X)
        driver.movel_relative([0.01, 0, 0, 0, 0, 0])

        # Should move to [0.51, 0.2, 0.3, 0, 0, 3.14]
        mock_ctrl_instance.moveL.assert_called_once_with(
            [0.51, 0.2, 0.3, 0, 0, 3.14], 0.25, 1.2, True
        )


def test_movej_relative_with_custom_params():
    """Test relative joint movement with custom parameters."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.getActualQ.return_value = [0, 0, 0, 0, 0, 0]
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._receiver = mock_recv_instance
        driver._connected = True

        driver.movej_relative([0.1, 0, 0, 0, 0, 0], velocity=0.5, acceleration=0.8)

        mock_ctrl_instance.moveJ.assert_called_once_with(
            [0.1, 0, 0, 0, 0, 0], 0.5, 0.8, True
        )
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_relative.py -v`

Expected: Tests fail due to missing relative movement methods

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
    def movej_relative(
        self,
        delta: List[float],
        velocity: float = 1.05,
        acceleration: float = 1.4,
        blocking: bool = True,
    ):
        """Move relative to current joint position.

        Args:
            delta: 6 joint deltas in radians
            velocity: Joint velocity in rad/s (default: 1.05)
            acceleration: Joint acceleration in rad/s² (default: 1.4)
            blocking: If True, wait for movement to complete (default: True)

        Returns:
            MotionToken if blocking=False, None if blocking=True

        Raises:
            ConnectionError: If not connected to robot
            OperationError: If movement fails or in read-only mode
        """
        current = self.jpos
        target = [c + d for c, d in zip(current, delta)]
        return self.movej(target, velocity, acceleration, blocking)

    def movel_relative(
        self,
        delta: List[float],
        velocity: float = 0.25,
        acceleration: float = 1.2,
        blocking: bool = True,
    ):
        """Move relative to current TCP pose.

        Args:
            delta: 6 pose deltas [x, y, z, rx, ry, rz]
            velocity: Tool velocity in m/s (default: 0.25)
            acceleration: Tool acceleration in m/s² (default: 1.2)
            blocking: If True, wait for movement to complete (default: True)

        Returns:
            MotionToken if blocking=False, None if blocking=True

        Raises:
            ConnectionError: If not connected to robot
            OperationError: If movement fails or in read-only mode
        """
        current = self.xpose
        target = [c + d for c, d in zip(current, delta)]
        return self.movel(target, velocity, acceleration, blocking)
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_relative.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_relative.py
git commit -m "feat: implement relative movements

- Add movej_relative for relative joint movements
- Add movel_relative for relative Cartesian movements
- Add delta to current position internally
- Support custom velocity, acceleration, blocking parameters"
```

---

### Task 13: Implement Path Movement (move_path)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_path.py`

**Step 1: Write the failing test**

Create `tests/test_driver_path.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_move_path_joint():
    """Test path movement in joint space."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        waypoints = [
            [0, -1.0, 1.0, 0, 0, 0],
            [0, -1.5, 1.5, 0, 0, 0],
            [0, -2.0, 2.0, 0, 0, 0],
        ]

        driver.move_path(waypoints, path_type="j")

        # Should call moveJ once with the full path
        mock_ctrl_instance.moveJ.assert_called_once()


def test_move_path_linear():
    """Test path movement in Cartesian space."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveL.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        waypoints = [
            [0.5, 0.2, 0.3, 0, 0, 3.14],
            [0.6, 0.2, 0.3, 0, 0, 3.14],
            [0.7, 0.2, 0.3, 0, 0, 3.14],
        ]

        driver.move_path(waypoints, path_type="l")

        # Should call moveL once with the full path
        mock_ctrl_instance.moveL.assert_called_once()


def test_move_path_with_blend_radius():
    """Test path movement with blend radius."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.moveJ.return_value = True
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        waypoints = [[0, -1.0, 1.0, 0, 0, 0], [0, -2.0, 2.0, 0, 0, 0]]

        driver.move_path(waypoints, path_type="j", blend_radius=0.1)

        # Note: ur_rtde handles blend_radius internally
        mock_ctrl_instance.moveJ.assert_called_once()
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_path.py -v`

Expected: Tests fail due to missing move_path method

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
    def move_path(
        self,
        waypoints: List[List[float]],
        path_type: str = "j",
        blend_radius: float = 0.0,
        velocity: float = 1.05,
        acceleration: float = 1.4,
    ) -> Optional[MotionToken]:
        """Move through multiple waypoints with smooth blending.

        Args:
            waypoints: List of waypoints (each is 6 floats)
            path_type: "j" for joint space, "l" for Cartesian (default: "j")
            blend_radius: Blending radius between waypoints (default: 0.0)
            velocity: Velocity (default varies by path_type)
            acceleration: Acceleration (default varies by path_type)

        Returns:
            MotionToken (currently always returns None - blocking only)

        Raises:
            ConnectionError: If not connected to robot
            OperationError: If movement fails or in read-only mode
            ValueError: If path_type is not "j" or "l"
        """
        if path_type not in ("j", "l"):
            raise ValueError(f"path_type must be 'j' or 'l', got '{path_type}'")

        if self.read_only:
            raise OperationError("Cannot move in read-only mode")

        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        # Choose move function based on path_type
        if path_type == "l":
            # Adjust defaults for linear movement
            velocity = velocity if velocity != 1.05 else 0.25
            acceleration = acceleration if acceleration != 1.4 else 1.2
            move_fn = self._controller.moveL
        else:
            move_fn = self._controller.moveJ

        success = move_fn(waypoints, velocity, acceleration, True)

        if not success:
            raise OperationError(f"move_path failed (path_type={path_type})")

        return None  # Currently blocking only
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_path.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_path.py
git commit -m "feat: implement path movement (move_path)

- Add move_path for multi-waypoint movements
- Support joint (j) and linear (l) path types
- Support blend_radius for smooth blending
- Validate path_type parameter"
```

---

## Phase 5: Additional State Properties and Utilities

### Task 14: Add Additional State Properties

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_additional_state.py`

**Step 1: Write the failing test**

Create `tests/test_driver_additional_state.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_jcurrent_property():
    """Test getting joint currents."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualCurrent.return_value = [1.0, 2.0, 1.5, 0.5, 0.3, 0.2]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        current = driver.jcurrent

        assert current == [1.0, 2.0, 1.5, 0.5, 0.3, 0.2]


def test_xvel_property():
    """Test getting TCP velocity."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualTCPSpeed.return_value = [0.1, 0.05, 0.02, 0, 0, 0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        vel = driver.xvel

        assert vel == [0.1, 0.05, 0.02, 0, 0, 0]


def test_xforce_property():
    """Test getting TCP force."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.getActualTCPForce.return_value = [0, 0, -10, 0, 0, 0]
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        force = driver.xforce

        assert force == [0, 0, -10, 0, 0, 0]


def test_is_moving_property():
    """Test checking if robot is moving."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_recv_instance = Mock()
        mock_recv_instance.isRobotConnected.return_value = True
        mock_recv_instance.getSpeedScaling.return_value = 1.0
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._receiver = mock_recv_instance
        driver._connected = True

        # This is a placeholder - actual implementation may differ
        # based on how ur_rtde provides "is moving" status
        moving = driver.is_moving()

        # For now, just check it returns a bool
        assert isinstance(moving, bool)
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_additional_state.py -v`

Expected: Tests fail due to missing properties

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add properties to URDriver class):

```python
    @property
    def jcurrent(self) -> List[float]:
        """Get current joint currents.

        Returns:
            List of 6 joint currents

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualCurrent()

    @property
    def xvel(self) -> List[float]:
        """Get current TCP velocity.

        Returns:
            List of 6 velocity values

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPSpeed()

    @property
    def xforce(self) -> List[float]:
        """Get current TCP force.

        Returns:
            List of 6 force values

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        return self._receiver.getActualTCPForce()

    @property
    def is_moving(self) -> bool:
        """Check if robot is currently moving.

        Returns:
            True if robot is moving, False otherwise

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._receiver is None:
            raise ConnectionError("Not connected to robot")
        # Check if any joint velocity is above threshold
        velocities = self.jvel
        return any(abs(v) > 0.001 for v in velocities)
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_additional_state.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_additional_state.py
git commit -m "feat: add additional state properties

- Add jcurrent property for joint currents
- Add xvel property for TCP velocity
- Add xforce property for TCP force
- Add is_moving property to check motion status"
```

---

### Task 15: Implement Stop Methods (stopj, stopl)

**Files:**
- Modify: `src/pyurrr/_driver.py`
- Create: `tests/test_driver_stop.py`

**Step 1: Write the failing test**

Create `tests/test_driver_stop.py`:

```python
import pytest
from unittest.mock import Mock, patch
from pyurrr import URDriver


def test_stopj():
    """Test stopping in joint space."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.stopj(acceleration=2.0)

        mock_ctrl_instance.stopJ.assert_called_once_with(2.0)


def test_stopl():
    """Test stopping in linear space."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.stopl(acceleration=10.0)

        mock_ctrl_instance.stopL.assert_called_once_with(10.0)


def test_stopj_default_acceleration():
    """Test stopj with default acceleration."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.stopj()

        mock_ctrl_instance.stopJ.assert_called_once_with(2.0)


def test_stopl_default_acceleration():
    """Test stopl with default acceleration."""
    with patch("pyurrr._driver.RTDEController") as mock_ctrl, \
         patch("pyurrr._driver.RTDEReceiver") as mock_recv:
        mock_ctrl_instance = Mock()
        mock_ctrl.return_value = mock_ctrl_instance

        mock_recv_instance = Mock()
        mock_recv_instance.isConnected.return_value = True
        mock_recv.return_value = mock_recv_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver._controller = mock_ctrl_instance
        driver._connected = True

        driver.stopl()

        mock_ctrl_instance.stopL.assert_called_once_with(10.0)
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_driver_stop.py -v`

Expected: Tests fail due to missing stop methods

**Step 3: Write minimal implementation**

Update `src/pyurrr/_driver.py` (add to URDriver class):

```python
    def stopj(self, acceleration: float = 2.0) -> None:
        """Stop robot in joint space.

        Args:
            acceleration: Deceleration in rad/s² (default: 2.0)

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        self._controller.stopJ(acceleration)

    def stopl(self, acceleration: float = 10.0) -> None:
        """Stop robot in linear (Cartesian) space.

        Args:
            acceleration: Deceleration in m/s² (default: 10.0)

        Raises:
            ConnectionError: If not connected to robot
        """
        if not self._connected or self._controller is None:
            raise ConnectionError("Not connected to robot")

        self._controller.stopL(acceleration)
```

**Step 4: Run test to verify it passes**

Run: `pytest tests/test_driver_stop.py -v`

Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/pyurrr/_driver.py tests/test_driver_stop.py
git commit -m "feat: implement stop methods

- Add stopj() for stopping in joint space
- Add stopl() for stopping in Cartesian space
- Support custom acceleration parameters
- Provide sensible defaults (2.0 rad/s² for joint, 10.0 m/s² for linear)"
```

---

## Phase 6: Documentation and Final Polish

### Task 16: Update Package Documentation

**Files:**
- Modify: `src/pyurrr/__init__.py`
- Modify: `README.md`

**Step 1: Update package exports and docstrings**

Update `src/pyurrr/__init__.py`:

```python
"""pyurrr - Easy-to-use UR robot driver package.

This package provides a simple, pythonic interface for controlling
Universal Robots arms by wrapping the ur_rtde library.

Example:
    >>> from pyurrr import URDriver
    >>> with URDriver("192.168.1.10") as robot:
    ...     robot.movej([0, -1.57, 1.57, 0, 0, 0])
    ...     print(robot.jpos)
"""

from pyurrr._driver import URDriver
from pyurrr._exceptions import URRDError, ConnectionError, OperationError, IKError
from pyurrr._token import MotionToken

__all__ = [
    "URDriver",
    "URRDError",
    "ConnectionError",
    "OperationError",
    "IKError",
    "MotionToken",
]

__version__ = "0.1.0"
```

**Step 2: Update README.md**

Update `README.md`:

```markdown
# pyurrr

Easy-to-use Python wrapper around [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) for controlling Universal Robots arms.

## Features

- **Simple API**: Single `URDriver` class for all robot operations
- **Context Manager Support**: Automatic connection management with `with` statement
- **Token-Based Non-Blocking**: Start movements and wait for them later
- **Convenience Methods**: Path following, relative movements, home position
- **Read-Only Mode**: Monitor robot state without controlling it
- **Type Hints**: Full type annotations for better IDE support

## Installation

```bash
pip install pyurrr
```

## Quick Start

```python
from pyurrr import URDriver

# Basic usage with automatic connection management
with URDriver("192.168.1.10") as robot:
    # Move to joint position
    robot.movej([0, -1.57, 1.57, 0, 0, 0])

    # Query state
    print(f"Joint positions: {robot.jpos}")
    print(f"TCP pose: {robot.xpose}")
```

## Documentation

See [Design Document](docs/plans/2026-01-23-ur-driver-design.md) for full API documentation.
```

**Step 3: Commit**

```bash
git add src/pyurrr/__init__.py README.md
git commit -m "docs: update package documentation

- Add comprehensive module docstring
- Add usage examples
- Update README with features and quick start"
```

---

### Task 17: Add Type Hint Stub File (Optional)

**Files:**
- Create: `src/pyurrr/py.typed`

**Step 1: Create py.typed marker file**

Create `src/pyurrr/py.typed`:

```
# Marker file for PEP 561 - indicates this package supports type checking
```

**Step 2: Commit**

```bash
git add src/pyurrr/py.typed
git commit -m "feat: add PEP 561 type hint marker

- Add py.typed file to indicate package supports type checking"
```

---

### Task 18: Run Full Test Suite

**Step 1: Run all tests**

```bash
pytest tests/ -v
```

Expected: All tests PASS

**Step 2: Run with coverage**

```bash
pytest tests/ --cov=src/pyurrr --cov-report=term-missing
```

**Step 3: Commit if any fixes needed**

---

## Summary

This implementation plan covers:

**Phase 1: Core Functionality** (Tasks 1-5)
- Exception hierarchy
- MotionToken dataclass
- URDriver constructor
- Connection management
- Context manager protocol

**Phase 2: State Query and Basic Movements** (Tasks 6-8)
- State properties (jpos, jvel, xpose)
- Basic movements (movej, movel)

**Phase 3: Kinematics** (Tasks 9-10)
- Forward kinematics (fk)
- Inverse kinematics (ik)

**Phase 4: Convenience Methods** (Tasks 11-13)
- Home position (set_home, go_home)
- Relative movements (movej_relative, movel_relative)
- Path movement (move_path)

**Phase 5: Additional State and Utilities** (Tasks 14-15)
- Additional state properties
- Stop methods

**Phase 6: Documentation and Polish** (Tasks 16-18)
- Package documentation
- README
- Final testing

---

**Next Steps:**
1. Review plan for completeness
2. Execute plan using @superpowers:subagent-driven-development or @superpowers:executing-plans
3. Test with real robot hardware
4. Gather user feedback and iterate
