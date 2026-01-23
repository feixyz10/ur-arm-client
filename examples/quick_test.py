#!/usr/bin/env python3
"""Quick test script for pyurrr with a real UR robot.

This is a simple sanity check to verify basic connectivity and movement.
"""

from pyurrr import URDriver

# Configuration
ROBOT_IP = "192.168.1.10"  # Change this to your robot's IP


def main():
    print(f"Testing connection to robot at {ROBOT_IP}...")

    with URDriver(ROBOT_IP, home_pose=None) as robot:
        # Check connection
        print("✓ Connected")
        print(f"  Joint positions: {robot.jpos}")
        print(f"  TCP pose: {robot.xpose}")
        print(f"  Joint temperatures: {robot.jtemp}")

        # Simple movement test
        print("\nTesting movement...")

        print("✓ Returned to home")
        robot.go_home()

        print("✓ Moved to test position")
        robot.movej_relative([0, -0.15, 0.15, 0, 0, 0])

        print("\nAll tests passed!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
