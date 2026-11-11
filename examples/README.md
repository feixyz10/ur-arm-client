# Examples

See the [User Guide](../USER_GUIDE.md) for the library API and controller setup. Run these commands from the repository root after `uv sync`.

## Read-only connection check

Replace the example address with your robot controller's IP. This script opens only the receive connection and prints joint positions, the tool pose, and joint temperatures.

```bash
uv run python examples/quick_test.py 192.168.1.10
```

## Manual motion check

`test_real_robot.py` opens a control connection, makes a non-blocking joint move, waits for it to finish, and returns to a configured home position. It prints the configured targets and asks you to confirm before connecting. Run `--help` for the arguments:

```bash
uv run python examples/test_real_robot.py --help
```

Supply two sets of six joint angles in radians and choose a velocity and acceleration that are appropriate for your setup. Verify the entire travel path from the arm's current pose before confirming. This script is separate from the offline test suite.
