# UR Robot Driver Package Design

**Date:** 2026-01-23
**Target Users:** Researchers/Prototypers (with robustness considerations)
**Goal:** Easy-to-use Python wrapper around ur_rtde library

---

## Overview

The `pyurrr` package provides a simple, pythonic interface for controlling Universal Robots arms. It wraps the `ur_rtde` library with a single unified `URDriver` class that supports context manager usage for automatic resource management.

**Design Principles:**
- Simplicity first - minimal boilerplate for common tasks
- Always raise exceptions on failures (no silent None returns)
- Clean disconnect on `with` block exit
- Token-based non-blocking movement API

---

## Package Structure

```
src/pyurrr/
  __init__.py          # Public API: exports URDriver class and exceptions
  _driver.py           # Core URDriver implementation
  _exceptions.py       # Custom exception hierarchy
  _token.py            # MotionToken dataclass
```

---

## Architecture

### The URDriver Class

A single unified class that:
- Connects to both `RTDEControlInterface` and `RTDEReceiveInterface` by default
- Supports read-only mode for monitoring without control
- Implements context manager protocol (`__enter__`, `__exit__`) for `with` statement support
- Wraps ur_rtde methods with cleaner, pythonic names
- Raises descriptive exceptions on all failures

### Connection Strategy

- **Lazy by default**: Doesn't connect until first use OR when entering `with` block
- **Reconnect support**: `connect()` and `disconnect()` methods for explicit control
- **Connection check**: `is_connected` property returns connection status

### Exception Hierarchy

```
URRDError               # Base exception
  ├─ ConnectionError    # Connection/disconnection failures
  ├─ OperationError     # Movement/operation failures
  └─ IKError           # Inverse kinematics has no solution
```

---

## API Design

### Constructor

```python
URDriver(
    ip: str,
    read_only: bool = False,
    lazy: bool = True,
    frequency: int = 125,
    home_pose: list[float] | None = None,
) -> None
```

**Parameters:**
- `ip`: Robot IP address (required)
- `read_only`: If True, only connect receiver (no control capabilities)
- `lazy`: If True, don't connect until first use or `with` block entry
- `frequency`: RTDE receive frequency in Hz
- `home_pose`: Initial home position for `go_home()` (None = zero position)

---

### Connection & Lifecycle

```python
connect() -> None
    """Establish connection to robot."""

disconnect() -> None
    """Clean disconnect - stop motions and close connections."""

is_connected: bool  # Property - connection status
is_read_only: bool  # Property - whether in read-only mode
```

**Context Manager Usage:**
```python
with URDriver("192.168.1.10") as robot:
    robot.movej(target_jpos)  # Auto-connect on entry, auto-disconnect on exit
```

---

### Movement API

#### Joint Space Movement

```python
movej(
    target: list[float] | np.ndarray,
    velocity: float = 1.05,    # rad/s
    acceleration: float = 1.4, # rad/s²
    blocking: bool = True,
) -> MotionToken | None
```

#### Cartesian Space Movement

```python
movel(
    target: list[float] | np.ndarray,  # pose [x,y,z,rx,ry,rz]
    velocity: float = 0.25,    # m/s
    acceleration: float = 1.2, # m/s²
    blocking: bool = True,
) -> MotionToken | None
```

#### IK-Enabled Movements

```python
movej_ik(target_pose, v, a, blocking=True) -> MotionToken | None
    """Solve IK then movej (convenience method)."""

movel_fk(target_jpos, v, a, blocking=True) -> MotionToken | None
    """Solve FK then movel (convenience method)."""
```

#### Speed Control

```python
speedj(joint_velocities, acceleration=2.0, time=0.0) -> None
speedl(cartesian_velocity, acceleration=0.5, time=0.0) -> None
```

#### Stopping

```python
stopj(acceleration=2.0) -> None   # Stop in joint space
stopl(acceleration=10.0) -> None  # Stop in Cartesian (linear) space
```

---

### Token-Based Non-Blocking API

```python
# Non-blocking returns a token
token = robot.movej(target, blocking=False)
# ... do other work ...
robot.wait_done(token)  # Wait for this specific movement

# Wait for all queued movements
robot.wait_all_done() -> None

# Check motion status
robot.is_moving() -> bool
```

**MotionToken:**
- Simple dataclass storing movement handle
- Used with `wait_done()` for specific movement synchronization

---

### Convenience Methods

#### Path Execution

```python
move_path(
    waypoints: list[list[float]],
    path_type: "j" | "l" = "j",
    blend_radius: float = 0.0,
    velocity: float = 1.05,
    acceleration: float = 1.4,
) -> MotionToken
    """Move through multiple waypoints with smooth blending."""
```

#### Relative Movements

```python
movej_relative(delta_jpos, v, a, blocking=True) -> MotionToken
    """Move relative to current joint positions."""

movel_relative(delta_pose, v, a, blocking=True) -> MotionToken
    """Move relative to current TCP pose."""
```

#### Home Position

```python
# Set via constructor
URDriver(ip, home_pose=None, ...)  # None means zero position

# Or set at runtime
set_home(pose: list[float] | None) -> None
    """Set home position. None resets to zero position."""

go_home(velocity=1.05, acceleration=1.4) -> MotionToken
    """Move to the configured home position."""
```

#### Teaching/Recording (via properties)

```python
# Teaching is done via properties, not separate methods
waypoint1 = robot.jpos  # Record current joint position
waypoint2 = robot.xpose  # Record current TCP pose
```

---

### State Query Properties

#### Joint State

```python
robot.jpos     # Joint positions [6x]
robot.jvel     # Joint velocities [6x]
robot.jcurrent # Joint currents [6x]
robot.jtemp    # Joint temperatures [6x] (if available)
```

#### Cartesian State

```python
robot.xpose   # TCP pose [x,y,z,rx,ry,rz]
robot.xvel    # TCP speed
robot.xforce  # TCP force
```

#### Robot Status

```python
robot.is_connected          # Connection status
robot.is_moving             # Currently in motion
robot.is_protective_stopped # Robot in protective stop
robot.is_emergency_stopped  # Emergency stop active
robot.mode                  # Robot operating mode
robot.safety_status         # Safety status bits
```

---

### Kinematics

```python
fk(jpos: list[float]) -> list[float]
    """Forward kinematics: joint positions -> TCP pose."""

ik(pose: list[float], jpos_hint: list[float] | None = None) -> list[float]
    """Inverse kinematics: TCP pose -> joint positions.
    Raises IKError if no solution (with hint, tries to find closest)."""
```

---

### Digital/IO (if supported by ur_rtde)

```python
# Outputs
get_digital_output(index: int) -> bool
set_digital_output(index: int, value: bool) -> None
get_analog_output(index: int) -> float
set_analog_output(index: int, value: float) -> None

# Inputs
get_digital_input(index: int) -> bool
get_analog_input(index: int) -> float
```

---

### Control Utilities

```python
stop_script() -> None
    """Stop the currently running script on robot."""

set_speed_scaling(factor: float) -> None
    """Set global speed scaling (0.0 to 1.0)."""

get_speed_scaling() -> float
    """Get current speed scaling factor."""
```

---

## Usage Examples

### Basic Usage

```python
from pyurrr import URDriver

# Simple with-statement usage
with URDriver("192.168.1.10") as robot:
    # Move to position
    target = [0, -1.57, 1.57, 0, 0, 0]
    robot.movej(target)

    # Query state
    print(f"Current joints: {robot.jpos}")
    print(f"TCP pose: {robot.xpose}")
```

### Non-Blocking Movements

```python
with URDriver("192.168.1.10") as robot:
    # Start movement
    token = robot.movej(target, blocking=False)

    # Do other work
    process_data()

    # Wait for completion
    robot.wait_done(token)
```

### Path Following

```python
waypoints = [
    [0, -1.0, 1.0, 0, 0, 0],
    [0, -1.5, 1.5, 0, 0, 0],
    [0, -2.0, 2.0, 0, 0, 0],
]

with URDriver("192.168.1.10") as robot:
    robot.move_path(waypoints, path_type="j", blend_radius=0.1)
```

### Relative Movements

```python
with URDriver("192.168.1.10") as robot:
    # Move 10cm in X direction relative to current pose
    robot.movel_relative([0.01, 0, 0, 0, 0, 0])
```

### Teaching Waypoints

```python
with URDriver("192.168.1.10") as robot:
    # Manually jog to positions and record
    print("Move to pose 1 and press Enter")
    input()
    pose1 = robot.xpose

    print("Move to pose 2 and press Enter")
    input()
    pose2 = robot.xpose

    # Replay
    robot.movel(pose1)
    robot.wait_all_done()
    robot.movel(pose2)
```

### Home Position

```python
# Set custom home in constructor
robot = URDriver("192.168.1.10", home_pose=[0, -1.57, 1.57, 0, 0, 0])

# Or set at runtime
robot.set_home([0, -1.0, 1.0, 0, 0, 0])

# Go to home
robot.go_home()
```

### Read-Only Monitoring

```python
with URDriver("192.168.1.10", read_only=True) as robot:
    while True:
        print(f"Joints: {robot.jpos}")
        print(f"Pose: {robot.xpose}")
        print(f"Force: {robot.xforce}")
        time.sleep(0.1)
```

---

## Implementation Priorities

### Phase 1: Core Functionality (Must Have)
- Connection management (connect, disconnect, is_connected)
- Basic movements (movej, movel)
- State query properties (jpos, xpose, etc.)
- Context manager support
- Exception hierarchy

### Phase 2: Standard Features (Should Have)
- Kinematics (fk, ik)
- Token-based non-blocking API (wait_done, wait_all_done, is_moving)
- Stopping (stopj, stopl)
- Speed control (speedj, speedl)
- Basic IO (digital/analog)

### Phase 3: Convenience Methods (Nice to Have)
- Path execution (move_path)
- Relative movements (movej_relative, movel_relative)
- Home position (set_home, go_home)
- Force mode (if ur_rtde supports it)
- Speed scaling

---

## Dependencies

- `ur-rtde` - Already bundled in third_party/
- `dataclasses` - For MotionToken and config (stdlib)

---

## Notes

- All movement methods accept both `list[float]` and `np.ndarray`
- All blocking operations default to `True` (safer for scripts)
- All methods raise exceptions on failure (no silent None returns)
- Exit from `with` block always calls `disconnect()` cleanly
- Read-only mode prevents calling control methods (raises appropriate error)
