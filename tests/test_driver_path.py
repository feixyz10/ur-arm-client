"""Test the native RTDE waypoint format and validation."""

from unittest.mock import patch

import pytest
from rtde_control import Path

from ur_arm_client import OperationError, URArmClient


@pytest.mark.parametrize("kind", ["j", "l"])
def test_path_sends_velocity_acceleration_and_intermediate_blend(kind):
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        move = getattr(control.return_value, f"move{kind.upper()}")
        points = [[0.1, 0, 0, 0, 0, 0], [0.2, 0, 0, 0, 0, 0]]

        assert client.move_path(points, kind, 0.25, 1.0, 0.01) is None

        assert points == [[0.1, 0, 0, 0, 0, 0], [0.2, 0, 0, 0, 0, 0]]
        move.assert_called_once()
        path, asynchronous = move.call_args.args
        assert asynchronous is False
        native_path = Path()
        append_path = (
            native_path.appendMovejPath if kind == "j" else native_path.appendMovelPath
        )
        append_path(path)
        commands = [
            line
            for line in native_path.toScriptCode().splitlines()
            if f"move{kind}(" in line
        ]
        assert len(commands) == 2
        assert "a=1,v=0.25,r=0.01" in commands[0]
        assert commands[1].endswith("a=1,v=0.25,r=0)")


@pytest.mark.parametrize(
    "waypoints, options",
    [
        ([[0] * 6], {"path_type": "x"}),
        ([], {}),
        ([[0] * 5], {}),
        ([[0] * 5 + [float("inf")]], {}),
        ([[0] * 6], {"velocity": 0}),
        ([[0] * 6], {"acceleration": float("nan")}),
        ([[0] * 6], {"blend_radius": -1}),
    ],
)
def test_invalid_paths_do_not_send_motion(waypoints, options):
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        with pytest.raises(ValueError):
            client.move_path(waypoints, **options)
        control.return_value.moveJ.assert_not_called()
        control.return_value.moveL.assert_not_called()


def test_path_reports_rejected_command():
    with (
        patch("ur_arm_client._client.RTDEController") as control,
        patch("ur_arm_client._client.RTDEReceiver"),
        URArmClient("robot") as client,
    ):
        control.return_value.moveJ.return_value = False
        with pytest.raises(OperationError, match="move_path failed"):
            client.move_path([[0] * 6])
