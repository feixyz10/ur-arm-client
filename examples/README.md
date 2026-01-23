# Examples

Example scripts for testing pyurrr with a real UR robot.

## Quick Start

**Quick Test** - `quick_test.py`
- Basic connectivity test
- Simple movement test
- Fast sanity check (5-10 seconds)
- Use this first to verify everything works

```bash
# Edit the script to set your robot IP
vim quick_test.py

# Run the quick test
python quick_test.py
```

## Comprehensive Test

**Full Test** - `test_real_robot.py`
- Tests all commonly-used features
- Connection, movement, kinematics, monitoring
- Takes 1-2 minutes to complete
- Comprehensive verification

```bash
# Edit the script to set your robot IP
vim test_real_robot.py

# Run the full test
python test_real_robot.py
```

## Before Running

1. Make sure the robot is powered on
2. Put robot in remote control mode
3. Ensure no emergency stop is active
4. Clear the workspace around the robot
5. Know your robot's IP address

## Safety

- These scripts move the robot at safe speeds
- Emergency stop is tested (will stop the robot)
- Keep hand near emergency stop button
- Always supervise robot movement

## Features Tested

The comprehensive test covers:
1. ✓ Connection and state queries
2. ✓ Joint space movement
3. ✓ Non-blocking movement with wait_done
4. ✓ Relative movements
5. ✓ Forward/inverse kinematics
6. ✓ State monitoring during motion
7. ✓ Home position feature
8. ✓ Emergency stop
