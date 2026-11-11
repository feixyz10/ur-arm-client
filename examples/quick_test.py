#!/usr/bin/env python3
"""Read joint and TCP state from a real robot without issuing commands."""

from argparse import ArgumentParser

from ur_arm_client import URArmClient


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("ip", help="Robot controller IP address")
    args = parser.parse_args()

    with URArmClient(args.ip, read_only=True) as arm:
        print(f"Joint positions (rad): {arm.jpos}")
        print(f"TCP pose (m, rad): {arm.xpose}")
        print(f"Joint temperatures (°C): {arm.jtemp}")


if __name__ == "__main__":
    main()
