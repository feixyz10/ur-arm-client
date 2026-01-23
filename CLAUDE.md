# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Commands

### Running Tests
```bash
# Run all tests
python -m pytest

# Run specific test file
python -m pytest tests/test_driver_connection.py

# Run tests matching a pattern
python -m pytest -k "test_connect"

# Run with verbose output
python -m pytest -v
```

### Linting and Formatting
```bash
# Check code style
python -m ruff check

# Format code
python -m ruff format

# Check and format in one command
python -m ruff check && python -m ruff format
```

### Building the Package
```bash
# Build the package (uses hatchling)
python -m build

# Install in development mode
pip install -e .
```

## Project Architecture

### High-Level Structure

This is a lightweight Python wrapper around the `ur_rtde` C++ library for controlling Universal Robots arms. The design prioritizes simplicity and pythonic API over comprehensive feature coverage.

**Key Design Principles:**
- Single unified `URDriver` class for all robot operations
- Context manager support for automatic resource management
- Token-based non-blocking movement API
- Always raises exceptions on failures (no silent None returns)
- Lazy connection by default (connects on first use or when entering `with` block)

### Package Structure

```
src/pyurrr/
  __init__.py          # Public API exports
  _driver.py           # Core URDriver implementation (main class)
  _exceptions.py       # Custom exception hierarchy
  _token.py            # MotionToken dataclass for non-blocking movements
```

### Core Components

**URDriver Class** (`_driver.py`):
- Wraps `rtde_control.RTDEControlInterface` and `rtde_receive.RTDEReceiveInterface`
- Supports read-only mode (monitoring without control)
- Implements context manager protocol (`__enter__`, `__exit__`)
- Properties for state queries: `jpos`, `jvel`, `xpose`, `xvel`, `xforce`, `is_moving`
- Movement methods: `movej`, `movel`, `movej_ik`, `movel_fk`, `speedj`, `speedl`
- Convenience methods: `move_path`, `movej_relative`, `movel_relative`, `go_home`
- Kinematics: `fk` (forward), `ik` (inverse)

**Exception Hierarchy** (`_exceptions.py`):
```
URRDError               # Base exception
  ├─ ConnectionError    # Connection/disconnection failures
  ├─ OperationError     # Movement/operation failures
  └─ IKError           # Inverse kinematics has no solution
```

**MotionToken** (`_token.py`):
- Simple dataclass for non-blocking movement synchronization
- Used with `wait_done()` for specific movement completion

### Testing Strategy

Tests are organized by feature in the `tests/` directory:
- `test_driver_constructor.py` - Constructor and initialization
- `test_driver_connection.py` - Connection management
- `test_driver_context_manager.py` - `with` statement behavior
- `test_driver_movej.py` - Joint space movement
- `test_driver_movel.py` - Cartesian space movement
- `test_driver_kinematics.py` - Forward/inverse kinematics
- `test_driver_path.py` - Multi-waypoint paths
- `test_driver_relative.py` - Relative movements
- `test_driver_home.py` - Home position management
- `test_driver_stop.py` - Stop methods
- `test_driver_state_query.py` - State properties
- `test_driver_additional_state.py` - Extended robot state
- `test_exceptions.py` - Exception hierarchy
- `test_token.py` - MotionToken functionality

**Important:** Tests use `unittest.mock` to mock `rtde_control` and `rtde_receive` modules, avoiding the need for a physical robot during development.

### Dependency Notes

**ur_rtde Library:**
- The C++ `ur_rtde` library is vendored in `third_party/ur_rtde/` (now removed from git)
- On macOS, it's installed from a custom fork via git: `git+https://gitee.com/i-my-fork/ur_rtde.git`
- On other platforms, it uses PyPI version `>=1.6.2`
- This is configured in `pyproject.toml` under `[tool.uv.sources]`

**Python Version:**
- Supports Python 3.9 to 3.12
- Uses `hatchling` as the build backend

### Connection Behavior

The driver has two connection modes:
1. **Lazy mode (default)**: Doesn't connect until first property access or method call, or when entering `with` block
2. **Eager mode**: Connects immediately in `__init__` when `lazy=False`

Connection state is managed internally and exposed via the `is_connected` property.

### Design Document

The full API specification is in `docs/plans/2026-01-23-ur-driver-design.md`. This document contains:
- Complete API reference
- Usage examples
- Implementation phases (Core → Standard → Convenience features)
- Design rationale

When adding new features, reference this document to understand the intended API surface.
