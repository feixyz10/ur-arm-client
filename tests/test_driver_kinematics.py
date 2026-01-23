"""Tests for kinematics methods (fk, ik)."""

import sys
import pytest
from unittest.mock import Mock, patch, MagicMock

# Mock the ur_rtde modules before importing pyurrr
sys.modules["rtde_control"] = MagicMock()
sys.modules["rtde_receive"] = MagicMock()

from pyurrr import URDriver, ConnectionError, IKError


def test_fk_returns_pose():
    """Test that fk returns TCP pose from joint positions."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
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

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        jpos = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        pose = driver.fk(jpos)

        assert pose == [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        mock_ctrl_instance.getForwardKinematics.assert_called_once_with(jpos)


def test_fk_requires_connection():
    """Test that fk raises ConnectionError when not connected."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        jpos = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        with pytest.raises(ConnectionError, match="Not connected to robot"):
            driver.fk(jpos)


def test_ik_with_solution():
    """Test that ik returns joint positions when solution exists."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getInverseKinematicsHasSolution.return_value = True
        mock_ctrl_instance.getInverseKinematics.return_value = [
            0.0,
            -1.57,
            1.57,
            0.0,
            0.0,
            0.0,
        ]
        mock_ctrl.return_value = mock_ctrl_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        pose = [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        jpos = driver.ik(pose)

        assert jpos == [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        mock_ctrl_instance.getInverseKinematicsHasSolution.assert_called_once_with(
            pose, None
        )
        mock_ctrl_instance.getInverseKinematics.assert_called_once_with(pose, None)


def test_ik_without_solution():
    """Test that ik raises IKError when no solution exists."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getInverseKinematicsHasSolution.return_value = False
        mock_ctrl.return_value = mock_ctrl_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        pose = [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        with pytest.raises(IKError, match="Inverse kinematics has no solution"):
            driver.ik(pose)


def test_ik_with_hint():
    """Test that ik uses jpos_hint parameter when provided."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        mock_ctrl_instance = Mock()
        mock_ctrl_instance.getInverseKinematicsHasSolution.return_value = True
        mock_ctrl_instance.getInverseKinematics.return_value = [
            0.1,
            -1.5,
            1.6,
            0.0,
            0.0,
            0.0,
        ]
        mock_ctrl.return_value = mock_ctrl_instance

        driver = URDriver(ip="192.168.1.10", lazy=True)
        driver.connect()

        pose = [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        jpos_hint = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        jpos = driver.ik(pose, jpos_hint)

        assert jpos == [0.1, -1.5, 1.6, 0.0, 0.0, 0.0]
        mock_ctrl_instance.getInverseKinematicsHasSolution.assert_called_once_with(
            pose, jpos_hint
        )
        mock_ctrl_instance.getInverseKinematics.assert_called_once_with(pose, jpos_hint)


def test_ik_requires_connection():
    """Test that ik raises ConnectionError when not connected."""
    with (
        patch("pyurrr._driver.RTDEController") as mock_ctrl,
        patch("pyurrr._driver.RTDEReceiver") as mock_recv,
    ):
        driver = URDriver(ip="192.168.1.10", lazy=True)

        pose = [0.5, 0.2, 0.3, 0.0, 0.0, 0.0]
        with pytest.raises(ConnectionError, match="Not connected to robot"):
            driver.ik(pose)
