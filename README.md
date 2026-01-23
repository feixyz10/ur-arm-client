# pyurrr

Easy-to-use Python wrapper around [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) for controlling Universal Robots arms.

**Python Version:** Requires Python 3.9 to 3.12 (inclusive)

## Features

- **Simple API** - Single `URDriver` class for all robot operations
- **Context Manager Support** - Automatic connection management with `with` statement
- **Blocking & Non-Blocking Moves** - Start movements and optionally wait for completion
- **Convenience Methods** - Path following, relative movements, home position
- **Read-Only Mode** - Monitor robot state without controlling it
- **Type Hints** - Full type annotations for better IDE support

## Installation

### Requirements

- Python 3.9, 3.10, 3.11, or 3.12
- `ur_rtde` library (automatically installed)

### Install from Wheel

Download the wheel and install:

```bash
pip install pyurrr-0.1.0-py3-none-any.whl
```

**Note:** The wheel enforces Python version constraints. Installation will fail on Python 3.13+.

## Quick Start

```python
from pyurrr import URDriver

# Basic usage with automatic connection management
with URDriver("192.168.1.10") as robot:
    # Move to joint position (radians)
    robot.movej([0, -1.57, 1.57, 0, 0, 0])

    # Query state
    print(f"Joint positions: {robot.jpos}")
    print(f"TCP pose: {robot.xpose}")
```

## Usage Examples

### Basic Movement

```python
from pyurrr import URDriver

with URDriver("192.168.1.10") as robot:
    # Joint space movement (rad/s, rad/s²)
    robot.movej([0, -1.57, 1.57, 0, 0, 0], velocity=1.05, acceleration=1.4)

    # Cartesian space movement (m/s, m/s²)
    # Pose format: [x, y, z, rx, ry, rz] in meters and radians
    robot.movel([0.3, 0.2, 0.4, 0, 3.14, 0], velocity=0.25, acceleration=1.2)
```

### Non-Blocking Movements

```python
from pyurrr import URDriver

with URDriver("192.168.1.10") as robot:
    # Start movement without waiting
    robot.movej([0, -1.57, 1.57, 0, 0, 0], blocking=False)

    # Do other work while robot moves
    print("Robot is moving...")

    # Wait for movement to complete
    while robot.is_moving:
        print(f"Current joints: {robot.jpos}")
```

### Path Following

```python
from pyurrr import URDriver

waypoints = [
    [0, -1.0, 1.0, 0, 0, 0],
    [0, -1.5, 1.5, 0, 0, 0],
    [0, -2.0, 2.0, 0, 0, 0],
]

with URDriver("192.168.1.10") as robot:
    # Execute path with smooth blending between waypoints
    robot.move_path(waypoints, path_type="j", blend_radius=0.1)
```

### Relative Movements

```python
from pyurrr import URDriver

with URDriver("192.168.1.10") as robot:
    # Move 10cm in X direction (Cartesian)
    robot.movel_relative([0.1, 0, 0, 0, 0, 0])

    # Rotate shoulder joint by 0.1 radians
    robot.movej_relative([0.1, 0, 0, 0, 0, 0])
```

### Home Position

```python
from pyurrr import URDriver

# Set custom home position
with URDriver("192.168.1.10", home_pose=[0, -1.57, 1.57, 0, 0, 0]) as robot:
    # Move around
    robot.movej([0, -1.0, 1.0, 0, 0, 0])

    # Return to home
    robot.go_home()

# Or set home at runtime
with URDriver("192.168.1.10") as robot:
    robot.set_home([0, -1.57, 1.57, 0, 0, 0])
    robot.go_home()
```

### Kinematics

```python
from pyurrr import URDriver

with URDriver("192.168.1.10") as robot:
    # Forward kinematics: joints -> pose
    jpos = [0, -1.57, 1.57, 0, 0, 0]
    pose = robot.fk(jpos)
    print(f"TCP pose: {pose}")

    # Inverse kinematics: pose -> joints
    target_pose = [0.3, 0.2, 0.4, 0, 3.14, 0]
    jpos = robot.ik(target_pose)
    robot.movej(jpos)
```

### Read-Only Monitoring

```python
from pyurrr import URDriver
import time

# Monitor without controlling
with URDriver("192.168.1.10", read_only=True) as robot:
    while True:
        print(f"Joints: {robot.jpos}")
        print(f"Pose: {robot.xpose}")
        print(f"Force: {robot.xforce}")
        time.sleep(0.1)
```

### Emergency Stop

```python
from pyurrr import URDriver

with URDriver("192.168.1.10") as robot:
    robot.movej([0, -1.57, 1.57, 0, 0, 0], blocking=False)

    # Emergency stop in joint space
    robot.stopj(acceleration=2.0)

    # Or stop in Cartesian (linear) space
    robot.stopl(acceleration=10.0)
```

### State Properties

```python
from pyurrr import URDriver

with URDriver("192.168.1.10") as robot:
    # Joint state
    robot.jpos      # Joint positions [6x]
    robot.jvel      # Joint velocities [6x]
    robot.jcurrent  # Joint currents [6x]

    # Cartesian state
    robot.xpose     # TCP pose [x,y,z,rx,ry,rz]
    robot.xvel      # TCP speed
    robot.xforce    # TCP force

    # Robot status
    robot.is_connected  # Connection status
    robot.is_moving     # Currently in motion
```

## API Reference

### Constructor

```python
URDriver(
    ip: str,                    # Robot IP address (required)
    read_only: bool = False,    # Monitor only, no control
    lazy: bool = True,          # Defer connection until first use
    frequency: int = 125,       # RTDE receive frequency (Hz)
    home_pose: list[float] | None = None,  # Home position
)
```

### Movement Methods

| Method | Description |
|--------|-------------|
| `movej(target, v, a, blocking)` | Joint space movement |
| `movel(target, v, a, blocking)` | Cartesian linear movement |
| `move_path(waypoints, type, blend)` | Multi-waypoint path |
| `movej_relative(delta, v, a)` | Relative joint movement |
| `movel_relative(delta, v, a)` | Relative Cartesian movement |
| `go_home(v, a)` | Move to home position |
| `stopj(acceleration)` | Stop in joint space |
| `stopl(acceleration)` | Stop in Cartesian space |

### Kinematics

| Method | Description |
|--------|-------------|
| `fk(jpos)` | Forward kinematics (joints → pose) |
| `ik(pose, hint)` | Inverse kinematics (pose → joints) |

### Connection

```python
robot.connect()           # Explicit connect
robot.disconnect()        # Explicit disconnect
robot.is_connected        # Connection status
```

## Coordinate System

- **Joint positions:** 6 values in radians [base, shoulder, elbow, wrist1, wrist2, wrist3]
- **TCP pose:** 6 values [x, y, z, rx, ry, rz] where position is meters and rotation is axis-angle radians

## Error Handling

All methods raise exceptions on failure:

```python
from pyurrr import URDriver, ConnectionError, OperationError, IKError

try:
    with URDriver("192.168.1.10") as robot:
        robot.ik([999, 999, 999, 0, 0, 0])  # Unreachable pose
except IKError as e:
    print(f"No IK solution: {e}")
except ConnectionError as e:
    print(f"Connection failed: {e}")
except OperationError as e:
    print(f"Operation failed: {e}")
```

## License

This package wraps `ur_rtde`. Refer to the ur_rtde license for underlying library terms.
