# ur-arm-client

A Python client for reading state and controlling Universal Robots arms over RTDE. Requires CPython 3.10 or later and compatible [ur-rtde-py](https://pypi.org/project/ur-rtde-py/) wheels.

See the [User Guide](USER_GUIDE.md) for installation, connection setup, motion examples, and troubleshooting. The package uses the same guide for its PyPI project description.

## Develop locally

```bash
uv sync
uv run pytest
uv build
```

Use `uv build --wheel` if you only need the wheel. The default test suite uses mocked robot connections. See [the example scripts](examples/README.md) for manual checks with a real controller.
