# ur-arm-client user guide

`ur-arm-client` is a Python client for Universal Robots arms. Use `URArmClient` to read joint and tool state, send joint or linear moves, and follow waypoint paths over RTDE. The client uses [ur-rtde-py](https://pypi.org/project/ur-rtde-py/) for the connection to the robot.

## Install

```bash
python -m pip install ur-arm-client
```

Use standard CPython 3.10 or later on a platform with compatible RTDE wheels: Linux x86-64 or ARM64, macOS Apple Silicon, or Windows x86-64. If you installed `ur-rtde` in the same environment, uninstall it first. Both distributions supply the same Python modules.

Your computer must be able to reach the robot controller. See the [RTDE setup guide](https://sdurobotics.gitlab.io/ur_rtde/pages/getting_started/environment_setup.html) for controller settings and operating modes. This client does not power on the robot, release its brakes, or change its operating mode.

## Connect and read state

Start with a read-only session. Replace the IP address with the controller's address:

```python
from ur_arm_client import URArmClient

with URArmClient("192.168.1.10", read_only=True) as arm:
    print(arm.jpos)   # Six joint angles, in radians
    print(arm.xpose)  # TCP pose in the robot base frame
```

The `with` block connects on entry and closes the connection on exit. For a longer-lived client, use `connect()` and `disconnect()` explicitly. Passing `lazy=False` connects during construction. The `frequency` argument sets the requested receive rate in Hz (default: 125). Check `is_connected` for the status of the RTDE connections the client needs.

| Property | What it reports |
|---|---|
| `jpos`, `jvel` | Joint angles and angular velocities, in rad and rad/s |
| `jcurrent`, `jtemp` | Joint currents and temperatures, in A and °C |
| `xpose` | TCP pose `[x, y, z, rx, ry, rz]`: position in meters and rotation vector in radians |
| `xvel` | TCP linear and angular velocity, in m/s and rad/s |
| `xforce` | TCP force and torque, in N and N·m |
| `is_moving` | Whether measured joint speed exceeds a small threshold; useful for observation |

TCP means tool center point. Read-only mode opens just the receive interface. Motion commands and `fk()` require a control connection.

## Command motion

Use targets and motion limits validated for your robot, tooling, and workspace. `movej()` takes six joint angles; its velocity and acceleration are in rad/s and rad/s². `movel()` takes a TCP pose in the robot base frame; its velocity and acceleration are in m/s and m/s².

```python
def move_to_joint_target(robot_ip, target, velocity, acceleration):
    with URArmClient(robot_ip) as arm:
        arm.movej(target, velocity=velocity, acceleration=acceleration)
```

By default both calls block until RTDE returns. To do other work while a move runs, pass `blocking=False` and keep the returned token:

```python
def move_and_read_state(robot_ip, target, velocity, acceleration):
    with URArmClient(robot_ip) as arm:
        token = arm.movej(
            target, velocity=velocity, acceleration=acceleration, blocking=False
        )
        print(arm.jpos)
        arm.wait_done(token, timeout=30)
```

Use one caller per client. There is one pending move at a time and no command queue. `wait_all_done()` waits for that same pending move if you did not keep its token. A token belongs to the connection that issued it. Waiting confirms that the RTDE operation ended; it does not verify that the arm reached the requested target. A timeout does not cancel motion. `stopj()` and `stopl()` request controlled deceleration and invalidate a pending token. They are software motion commands, not safety-rated emergency stops.

## Paths, offsets, and home

- `move_path(waypoints, path_type="j", velocity=0.25, acceleration=1.0, blend_radius=0.0)` runs a blocking path. Give each waypoint six joint angles for `"j"` or a six-value TCP pose for `"l"`. Velocity and acceleration use the corresponding motion units. A nonzero blend radius, in meters, applies to intermediate waypoints; the final waypoint always uses zero blend.
- `movej_relative(delta, ...)` adds six angular offsets to the current joints. `movel_relative(delta, ...)` composes a pose offset in the current TCP frame. Its first three values are translations along the tool axes, in meters, and the last three form a rotation vector, in radians.
- Set a known home position with `home_pose=` at construction or with `set_home(joints)`, then call `go_home()`. Home is a six-angle joint target. If you leave it unset, the library defaults to `[0, -1.57, 1.57, 0, 0, 0]`; verify that configuration before using it on your robot.
- `fk(joints)` returns the TCP pose for six joint angles. Inverse kinematics is not part of this version.

## Troubleshooting

| Symptom | What to check |
|---|---|
| Install reports no compatible wheel | Check the Python implementation, CPU architecture, and wheels available for `ur-rtde-py` and its runtime dependency. |
| Connection fails | Check the robot IP, network access, and controller settings. Try a read-only session to verify the receive connection separately. |
| A command is rejected | Confirm the client was opened for control, the inputs are valid, and no previous non-blocking move is pending. |
| Waiting times out | Check the controller and arm state before retrying or sending another move. The original move may still be active. |

Catch `ur_arm_client.ConnectionError` for connection or cleanup failures, `OperationError` for rejected operations, and Python's `TimeoutError` for expired waits. Invalid path options or wait timeouts raise `ValueError`. The underlying RTDE bindings can also report their own errors.
