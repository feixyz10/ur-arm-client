# Repository guide

This repository provides `ur-arm-client`, a Python client for Universal Robots arms over RTDE. The import name is `ur_arm_client`, and `URArmClient` is the sole public client class.

The [User Guide](USER_GUIDE.md) is the source for the PyPI project description, via `project.readme` in `pyproject.toml`. Keep it focused on the public API and use absolute URLs for external links. Keep repository commands and example script instructions in their respective READMEs.

## Development

- Run `uv sync` to install the project and test dependencies.
- Run `uv run pytest` for offline unit tests, or `uv run pytest tests/test_driver_connection.py` for a single module. Tests use the installed RTDE bindings and mock the interfaces that connect to a robot.
- Run `uv build` for the source distribution and wheel, or `uv build --wheel` for only the wheel. Keep the package version in `pyproject.toml`. On recent uv versions, `uv version --bump patch` can update it.
- The minimum interpreter is CPython 3.10. Installability also depends on available `ur-rtde-py` and backend wheels for the platform. Do not install `ur-rtde` alongside `ur-rtde-py`; they export the same Python modules.

## Code and behavior

- `src/ur_arm_client/_driver.py` wraps `RTDEControlInterface` and `RTDEReceiveInterface`. By default, constructing a client does not open sockets. Call `connect()` or use a `with` block; `lazy=False` connects during construction. Read-only mode opens only the receive connection.
- `_token.py` contains the internal handle for a non-blocking move. RTDE operation IDs track completion. The client allows one outstanding move at a time and does not queue commands. A token belongs to the connection that issued it.
- `move_path()` uses RTDE waypoint entries with six coordinates followed by velocity, acceleration, and blend radius. The final waypoint has zero blend. `movel_relative()` composes offsets in the current TCP frame.
- `_exceptions.py` defines the public error types. Inverse kinematics is not implemented.
- `examples/` includes scripts that can move a physical arm. Keep those scripts out of automated test runs.
