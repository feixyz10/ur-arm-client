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
    print(arm.joint_positions["shoulder"])  # Shoulder angle, in radians
    print(arm.tcp_pose.position)            # TCP (x, y, z) in the robot base frame, in meters
```

The `with` block connects on entry and closes the connection on exit. For a longer-lived client, use `connect()` and `disconnect()` explicitly. Passing `lazy=False` connects during construction. The `frequency` argument sets the requested receive rate in Hz (default: 125). Check `is_connected` for the status of the RTDE connections the client needs.

| Property | What it reports |
|---|---|
| `joint_positions` | `JPos`: joint angles in rad |
| `joint_velocities` | `JVel`: joint angular velocities in rad/s |
| `joint_currents` | `JCurr`: joint currents in A |
| `joint_temperatures` | `JTemp`: joint temperatures in °C |
| `tcp_pose` | `Pose`: `.position` in m and `.rotvec` in rad, relative to the robot base |
| `tcp_velocity` | `Twist`: `.linear` in m/s and `.angular` in rad/s |
| `tcp_wrench` | `Wrench`: RTDE `actual_TCP_force` unchanged, with payload-compensated `.force` in N and `.torque` in N·m |
| `is_moving` | Whether measured joint speed exceeds a small threshold; useful for observation |

TCP means tool center point. Read-only mode opens just the receive interface. Motion and kinematics methods require a control connection.

## TCP and joint values

All seven telemetry values are immutable six-value sequences. Read a single value with an integer index, take a slice to get a tuple, or unpack all six values. `Pose.position` is `(x, y, z)`; `Pose.rotvec` is `(rx, ry, rz)` in rotation-vector form, not Euler angles.

`Twist` stores linear velocity followed by angular velocity; it contains no acceleration. `Wrench` stores force followed by torque. The client preserves the component order and coordinate conventions of [UR's RTDE outputs](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html).

`tcp_wrench` reports the controller's payload-compensated values in robot-base axes. It does not rotate the components into tool axes or shift the point about which torque is measured. For the built-in sensor, the [UR driver](https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Driver/main/ur_robot_driver/src/hardware_interface.cpp) treats the reference point as the tool flange on e-Series and the TCP on CB3. If you need a wrench expressed at the TCP in tool axes, account for your controller and sensor setup before transforming it.

```python
pose = arm.tcp_pose
x = pose[0]
xy = pose[:2]  # (x, y)
x, y, z, rx, ry, rz = pose

joints = arm.joint_positions
shoulder = joints.shoulder  # Also joints["shoulder"] or joints[1]
shoulder_rate = arm.joint_velocities["shoulder"]
linear_velocity = arm.tcp_velocity.linear
force = arm.tcp_wrench.force
```

All four joint telemetry types inherit from `JVec`. Their names and index order are `base`, `shoulder`, `elbow`, `wrist1`, `wrist2`, and `wrist3`. An unknown name raises `KeyError`.

Import `Pose` and `JPos` to construct your own targets. Both provide `from_sequence(values)` for six numbers in the order shown above.

```python
from ur_arm_client import JPos, Pose

pose = Pose(position=(0.5, 0.2, 0.3), rotvec=(0.0, 0.0, 0.0))
joints = JPos(base=0, shoulder=-1.57, elbow=1.57, wrist1=0, wrist2=0, wrist3=0)
```

### Migrating from 0.1.x

The old telemetry properties have been replaced:

| 0.1.x property | 0.2.0 property |
|---|---|
| `jpos` | `joint_positions` |
| `jvel` | `joint_velocities` |
| `jcurrent` | `joint_currents` |
| `jtemp` | `joint_temperatures` |
| `xpose` | `tcp_pose` |
| `xvel` | `tcp_velocity` |
| `xforce` | `tcp_wrench` |

Telemetry and `fk()` now return value objects rather than lists. Indexing, slicing, iteration, and unpacking still work; slices return tuples. Use `list(value)` for mutation, list comparisons, or serialization with `json.dumps()`. The library does not convert rotation representations.

## Command motion

Use targets and motion limits validated for your robot, tooling, and workspace. `movej()` takes six joint angles; its velocity and acceleration are in rad/s and rad/s². `movel()` takes a TCP pose in the robot base frame; its velocity and acceleration are in m/s and m/s².

`movej()` accepts a `JPos` value and `movel()` accepts a `Pose`. Both still accept lists or tuples of six numbers. Passing the wrong value type raises `OperationError`.

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
        print(arm.joint_positions)
        arm.wait_done(token, timeout=30)
```

Use one caller per client. There is one pending move at a time and no command queue. `wait_all_done()` waits for that same pending move if you did not keep its token. A token belongs to the connection that issued it. Waiting confirms that the RTDE operation ended; it does not verify that the arm reached the requested target. A timeout does not cancel motion. `stopj()` and `stopl()` request controlled deceleration and invalidate a pending token. They are software motion commands, not safety-rated emergency stops.

## Paths, offsets, and home

- `move_path(waypoints, path_type="j", velocity=0.25, acceleration=1.0, blend_radius=0.0)` runs a blocking path. Give each waypoint a `JPos` or six joint angles for `"j"`, and a `Pose` or six TCP pose values for `"l"`. Velocity and acceleration use the corresponding motion units. A nonzero blend radius, in meters, applies to intermediate waypoints; the final waypoint always uses zero blend.
- `movej_relative(delta, ...)` adds six angular offsets to the current joints. `movel_relative(delta, ...)` composes a pose offset in the current TCP frame. Its first three values are translations along the tool axes, in meters, and the last three form a rotation vector, in radians. Pass offsets as a list or tuple; telemetry value objects are not accepted as relative offsets.
- Set a known home position with `home_pose=` at construction or with `set_home(joints)`, then call `go_home()`. Home is a six-angle joint target. If you leave it unset, the library defaults to `[0, -1.57, 1.57, 0, 0, 0]`; verify that configuration before using it on your robot.

## Forward and inverse kinematics

These methods compute a pose or joint configuration on the controller without commanding motion:

| Method | Result |
|---|---|
| `fk(joints)` | A `Pose` for six joint angles or a `JPos` value. |
| `ik_check(pose, *, qnear=None)` | Whether the controller finds a kinematic solution. This does not check path safety or collisions. |
| `ik(pose, *, qnear=None)` | A `JPos` value on success, or `None` if the controller confirms there is no solution. |

Both IK methods accept a `Pose` or six TCP pose values in the robot base frame. Pass `qnear` as a `JPos` value or six joint angles in radians to select a solution near that configuration. If omitted, the controller uses its current joint configuration.

```python
def solve_pose(arm, target):
    joints = arm.ik(target, qnear=arm.joint_positions)
    if joints is None:
        print("No inverse kinematics solution")
    return joints
```

`ik()` calls the solver directly, with no version lookup or precheck on success. If it returns an empty result and the control script is still running, the client attempts to confirm whether there is a solution. An empty result alone can also indicate a failed RTDE command. Invalid inputs, communication failures, and unconfirmed failures raise exceptions. Directly solving an unreachable target may stop the control script on some controllers; use `ik_check()` to screen uncertain targets.

Both `ik_check()` and the confirmation after an empty result require PolyScope 3.15 or later on CB3, or 5.10 or later on newer controllers, as required by the [ur-rtde control script](https://gitlab.com/sdurobotics/ur_rtde/-/raw/master/scripts/rtde_control.script). The first check reads the firmware version over RTDE and caches it until disconnect. Unsupported firmware raises `OperationError`; a version query that cannot complete raises `ConnectionError`.

## Troubleshooting

| Symptom | What to check |
|---|---|
| Install reports no compatible wheel | Check the Python implementation, CPU architecture, and wheels available for `ur-rtde-py` and its runtime dependency. |
| Connection fails | Check the robot IP, network access, and controller settings. Try a read-only session to verify the receive connection separately. |
| A command is rejected | Confirm the client was opened for control, the inputs are valid, and no previous non-blocking move is pending. |
| Waiting times out | Check the controller and arm state before retrying or sending another move. The original move may still be active. |

Catch `ur_arm_client.ConnectionError` for connection or cleanup failures, `OperationError` for rejected operations, and Python's `TimeoutError` for expired waits. Invalid path options or wait timeouts raise `ValueError`. The underlying RTDE bindings can also report their own errors.
