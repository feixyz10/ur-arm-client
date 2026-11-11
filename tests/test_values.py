"""Public pose and joint value contracts."""

from collections.abc import Sequence
from dataclasses import FrozenInstanceError

import pytest

from ur_arm_client import JCurr, JPos, JTemp, JVec, JVel, Pose, Twist, Wrench

VALUES = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


@pytest.mark.parametrize(
    "value_type", [Pose, Twist, Wrench, JVec, JPos, JVel, JCurr, JTemp]
)
def test_values_are_immutable_six_element_sequences(value_type):
    source = list(VALUES)
    value = value_type.from_sequence(source)
    source[0] = 99

    assert isinstance(value, Sequence)
    assert len(value) == 6
    assert value[0] == VALUES[0]
    assert value[-1] == VALUES[-1]
    assert value[0:2] == VALUES[0:2]
    assert value[::-1] == VALUES[::-1]
    assert list(value) == list(VALUES)
    with pytest.raises(IndexError):
        _ = value[6]
    with pytest.raises(TypeError):
        value[0] = 99


@pytest.mark.parametrize(
    "value_type", [Pose, Twist, Wrench, JVec, JPos, JVel, JCurr, JTemp]
)
@pytest.mark.parametrize("size", [0, 5, 7])
def test_flat_construction_requires_six_values(value_type, size):
    with pytest.raises(ValueError, match="exactly 6"):
        value_type.from_sequence([0] * size)


def test_pose_fields_are_immutable_triples():
    pose = Pose(position=(0.1, 0.2, 0.3), rotvec=(0.4, 0.5, 0.6))

    assert pose.position == VALUES[:3]
    assert pose.rotvec == VALUES[3:]
    with pytest.raises(FrozenInstanceError):
        pose.position = (0, 0, 0)
    with pytest.raises(ValueError, match="rotvec requires exactly 3"):
        Pose((0, 0, 0), (0, 0))


@pytest.mark.parametrize("value_type", [JVec, JPos, JVel, JCurr, JTemp])
def test_joint_names_and_attribute_order(value_type):
    joints = value_type(*VALUES)
    names = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")

    for index, name in enumerate(names):
        assert joints[name] == getattr(joints, name) == VALUES[index]
    with pytest.raises(KeyError, match="gripper"):
        _ = joints["gripper"]
    with pytest.raises(FrozenInstanceError):
        joints.shoulder = 99


@pytest.mark.parametrize(
    "value_type, names", [(Twist, ("linear", "angular")), (Wrench, ("force", "torque"))]
)
def test_cartesian_telemetry_has_named_immutable_components(value_type, names):
    value = value_type.from_sequence(VALUES)

    assert getattr(value, names[0]) == VALUES[:3]
    assert getattr(value, names[1]) == VALUES[3:]
    with pytest.raises(FrozenInstanceError):
        setattr(value, names[0], (0.0, 0.0, 0.0))


def test_joint_types_share_behavior_but_are_distinct_values():
    positions = JPos.from_sequence(VALUES)
    velocities = JVel.from_sequence(VALUES)
    assert isinstance(positions, JVec)
    assert isinstance(velocities, JVec)
    assert not isinstance(velocities, JPos)
    assert positions != velocities
