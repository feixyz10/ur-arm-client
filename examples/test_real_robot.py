#!/usr/bin/env python3
"""Test script for pyurrr with a real UR robot.

This script tests commonly-used features with an actual robot.
Modify the ROBOT_IP variable to match your robot's IP address.
"""

import time
from pyurrr import URDriver

# Configuration
ROBOT_IP = "192.168.1.10"  # Change this to your robot's IP

# Test positions (safe positions that should work on most UR robots)
HOME_JOINTS = [0, -1.57, 1.57, 0, 0, 0]
TEST_JOINTS_1 = [0, -1.0, 1.0, 0, 0, 0]
TEST_JOINTS_2 = [0, -1.3, 1.3, 0, 0, 0]


def print_section(title: str) -> None:
    """Print a section header."""
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def test_connection():
    """Test basic connection and state queries."""
    print_section("1. Testing Connection")

    with URDriver(ROBOT_IP) as robot:
        print(f"✓ Connected to robot at {ROBOT_IP}")
        print(f"  Connection status: {robot.is_connected}")

        # Query current state
        print(f"\n  Current joint positions: {robot.jpos}")
        print(f"  Current TCP pose: {robot.xpose}")
        print(f"  Joint temperatures: {robot.jtemp}")


def test_joint_movement():
    """Test basic joint space movements."""
    print_section("2. Testing Joint Movement")

    with URDriver(ROBOT_IP) as robot:
        print("Moving to test position 1...")
        robot.movej(TEST_JOINTS_1, velocity=1.0, acceleration=1.0)
        print(f"✓ Reached position: {robot.jpos}")

        time.sleep(0.5)

        print("Moving to test position 2...")
        robot.movej(TEST_JOINTS_2, velocity=1.0, acceleration=1.0)
        print(f"✓ Reached position: {robot.jpos}")

        time.sleep(0.5)

        print("Returning to home...")
        robot.movej(HOME_JOINTS)
        print(f"✓ Back to home: {robot.jpos}")


def test_non_blocking_movement():
    """Test non-blocking movement with wait_done."""
    print_section("3. Testing Non-Blocking Movement")

    with URDriver(ROBOT_IP) as robot:
        print("Starting non-blocking movement...")

        # Start movement without waiting
        token = robot.movej(TEST_JOINTS_1, blocking=False)
        print("✓ Movement started (non-blocking)")

        # Do something else while moving
        for i in range(3):
            print(f"  Working on something else... ({i + 1}/3)")
            time.sleep(0.2)

        # Now wait for completion
        print("Waiting for movement to complete...")
        robot.wait_done(token)
        print(f"✓ Movement completed: {robot.jpos}")

        # Return to home
        robot.movej(HOME_JOINTS)


def test_relative_movement():
    """Test relative movements."""
    print_section("4. Testing Relative Movement")

    with URDriver(ROBOT_IP) as robot:
        # First go to a known position
        robot.movej(TEST_JOINTS_1)
        print(f"Starting position: {robot.jpos}")

        # Move relative to current position
        print("Moving relative by +0.1 rad on first joint...")
        robot.movej_relative([0.1, 0, 0, 0, 0, 0])
        print(f"✓ After relative move: {robot.jpos}")

        time.sleep(0.5)

        # Move back
        print("Moving relative by -0.1 rad on first joint...")
        robot.movej_relative([-0.1, 0, 0, 0, 0, 0])
        print(f"✓ After relative move back: {robot.jpos}")

        # Return to home
        robot.movej(HOME_JOINTS)


def test_kinematics():
    """Test forward and inverse kinematics."""
    print_section("5. Testing Kinematics")

    with URDriver(ROBOT_IP) as robot:
        # Test forward kinematics
        jpos = TEST_JOINTS_1
        print(f"Input joint positions: {jpos}")

        pose = robot.fk(jpos)
        print(f"Forward kinematics result: {pose}")

        # # Test inverse kinematics
        # print("\nComputing inverse kinematics...")
        # jpos_computed = robot.ik(pose, jpos_hint=jpos)
        # print(f"Inverse kinematics result: {jpos_computed}")

        # # Move to the computed position to verify
        # print("Moving to IK result...")
        # robot.movej(jpos_computed)
        # print(f"✓ Reached position: {robot.jpos}")

        # Return to home
        robot.movej(HOME_JOINTS)


def test_state_monitoring():
    """Test state monitoring while moving."""
    print_section("6. Testing State Monitoring")

    with URDriver(ROBOT_IP) as robot:
        print("Monitoring state during movement...")

        # Start a movement
        robot.movej(TEST_JOINTS_1, velocity=0.5, blocking=False)
        time.sleep(0.1)  # Wait for the robot to start moving

        # Monitor while moving
        start_time = time.time()
        while robot.is_moving and (time.time() - start_time) < 5:
            print(f"  Joints: {robot.jpos}")
            print(f"  Velocities: {robot.jvel}")
            print(f"  Temperatures: {robot.jtemp}")
            print(f"  Is moving: {robot.is_moving}")
            time.sleep(0.1)

        print("✓ Movement completed")
        robot.movej(HOME_JOINTS)


def test_home_position():
    """Test home position feature."""
    print_section("7. Testing Home Position")

    with URDriver(ROBOT_IP, home_pose=TEST_JOINTS_1) as robot:
        print(f"Home position set to: {robot.home_pose}")

        # Move away from home
        print("Moving away from home...")
        robot.movej(TEST_JOINTS_2)
        print(f"Current position: {robot.jpos}")

        # Return to home
        print("\nReturning to home...")
        robot.go_home()
        print(f"✓ Back to home: {robot.jpos}")

        # Reset to zero home
        robot.set_home(HOME_JOINTS)
        print(f"\nHome reset to: {robot.home_pose}")
        robot.go_home()
        print(f"✓ At new home: {robot.jpos}")


def test_emergency_stop():
    """Test emergency stop."""
    print_section("8. Testing Emergency Stop")

    with URDriver(ROBOT_IP) as robot:
        print("Starting slow movement...")

        # Start a very slow movement
        robot.movej(TEST_JOINTS_1, velocity=0.2, acceleration=0.5, blocking=False)

        # Let it move a bit
        time.sleep(0.5)

        # Emergency stop
        print("Triggering emergency stop...")
        robot.stopj(acceleration=5.0)
        print("✓ Emergency stop triggered")

        print(f"Stopped at: {robot.jpos}")

        # Return to home
        robot.movej(HOME_JOINTS)


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("  PYURRR REAL ROBOT TEST")
    print("=" * 60)
    print(f"\nRobot IP: {ROBOT_IP}")
    print("\nMake sure:")
    print("  1. Robot is powered on")
    print("  2. Robot is in remote control mode")
    print("  3. No emergency stop is active")
    print("  4. Robot has clear workspace")

    input("\nPress Enter to start tests...")

    try:
        test_connection()
        test_joint_movement()
        test_non_blocking_movement()
        test_relative_movement()
        test_state_monitoring()
        test_home_position()
        test_emergency_stop()

        print_section("All Tests Completed!")
        print("✓ All tests passed successfully")

    except KeyboardInterrupt:
        print("\n\n✗ Tests interrupted by user")
    except Exception as e:
        print(f"\n\n✗ Test failed with error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
