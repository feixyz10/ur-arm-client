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
