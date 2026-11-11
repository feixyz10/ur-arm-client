"""Tests for forward kinematics."""

import pytest
from unittest.mock import Mock, patch

from ur_arm_client import ConnectionError, JPos, Pose, URArmClient


@pytest.mark.parametrize("make_joints", [list, tuple, JPos.from_sequence])
def test_fk_returns_pose(make_joints):
    """Test that fk returns TCP pose from joint positions."""
    with (
        patch("ur_arm_client._client.RTDEController") as mock_ctrl,
        patch("ur_arm_client._client.RTDEReceiver"),
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getForwardKinematics.return_value = [
            0.5,
            0.2,
            0.3,
            0.0,
            0.0,
            0.0,
        ]
        mock_ctrl.return_value = mock_ctrl_instance

        driver = URArmClient(ip="192.168.1.10", lazy=True)
        driver.connect()

        jpos = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        pose = driver.fk(make_joints(jpos))

        assert pose == Pose((0.5, 0.2, 0.3), (0.0, 0.0, 0.0))
        mock_ctrl_instance.getForwardKinematics.assert_called_once_with(jpos)


def test_fk_requires_connection():
    """Test that fk raises ConnectionError when not connected."""
    with (
        patch("ur_arm_client._client.RTDEController"),
        patch("ur_arm_client._client.RTDEReceiver"),
    ):
        driver = URArmClient(ip="192.168.1.10", lazy=True)

        jpos = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        with pytest.raises(ConnectionError, match="Not connected to robot"):
            driver.fk(jpos)
