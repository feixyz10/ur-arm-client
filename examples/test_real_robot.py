#!/usr/bin/env python3
"""Manually check a non-blocking joint move and a return to home."""

from argparse import ArgumentParser

from ur_arm_client import URArmClient


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("ip", help="Robot controller IP address")
    parser.add_argument(
        "--target", nargs=6, type=float, required=True, help="Target angles (rad)"
    )
    parser.add_argument(
        "--home", nargs=6, type=float, required=True, help="Home angles (rad)"
    )
    parser.add_argument(
        "--velocity", type=float, required=True, help="Joint speed (rad/s)"
    )
    parser.add_argument(
        "--acceleration", type=float, required=True, help="Joint acceleration (rad/s²)"
    )
    args = parser.parse_args()

    print(
        f"Controller: {args.ip}\nJoint target: {args.target}\nHome target: {args.home}"
    )
    if input("Verify the motion path, then type MOVE to continue: ") != "MOVE":
        return

    with URArmClient(args.ip, home_pose=args.home) as arm:
        token = arm.movej(
            args.target,
            velocity=args.velocity,
            acceleration=args.acceleration,
            blocking=False,
        )
        arm.wait_done(token)
        print(f"Joint position after operation: {arm.jpos}")
        arm.go_home(velocity=args.velocity, acceleration=args.acceleration)


if __name__ == "__main__":
    main()
