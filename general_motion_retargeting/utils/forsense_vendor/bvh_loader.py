"""
Small standalone BVH loader for ForSense exports.

This module intentionally does not depend on the Xsens BVHParser. It supports
the two ForSense BVH shapes used by this repo:

- lower_snake_case exports rooted at ``hip``
- Mixamo-compatible CamelCase exports rooted at ``Hips``
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


CM_TO_M = 0.01


SNAKE_TO_GMR = {
    "hip": "Hips",
    "chest": "Chest4",
    "head": "Head",
    "left_shoulder": "LeftShoulder",
    "left_upper_arm": "LeftElbow",
    "left_lower_arm": "LeftWrist",
    "left_hand": "LeftHand",
    "right_shoulder": "RightShoulder",
    "right_upper_arm": "RightElbow",
    "right_lower_arm": "RightWrist",
    "right_hand": "RightHand",
    "left_upper_leg": "LeftHip",
    "left_lower_leg": "LeftKnee",
    "left_foot": "LeftFoot",
    "right_upper_leg": "RightHip",
    "right_lower_leg": "RightKnee",
    "right_foot": "RightFoot",
}


MIXAMO_TO_GMR = {
    "Hips": "Hips",
    "Spine2": "Chest4",
    "Head": "Head",
    "LeftShoulder": "LeftShoulder",
    "RightShoulder": "RightShoulder",
    "LeftArm": "LeftElbow",
    "LeftForeArm": "LeftWrist",
    "LeftHand": "LeftHand",
    "RightArm": "RightElbow",
    "RightForeArm": "RightWrist",
    "RightHand": "RightHand",
    "LeftUpLeg": "LeftHip",
    "LeftLeg": "LeftKnee",
    "LeftFoot": "LeftFoot",
    "RightUpLeg": "RightHip",
    "RightLeg": "RightKnee",
    "RightFoot": "RightFoot",
}


@dataclass(frozen=True)
class BVHSkeleton:
    names: list[str]
    offsets: np.ndarray
    parents: list[int]
    channels: list[list[str]]


def _parse_hierarchy(text: str, scale: float) -> BVHSkeleton:
    names: list[str] = []
    offsets: list[np.ndarray] = []
    parents: list[int] = []
    channels: list[list[str]] = []
    stack: list[int] = []

    for raw_line in text.splitlines():
        line = raw_line.strip()

        if line.startswith("ROOT") or line.startswith("JOINT"):
            joint_name = line.split()[1]
            parent_idx = stack[-1] if stack else -1
            names.append(joint_name)
            offsets.append(np.zeros(3))
            parents.append(parent_idx)
            channels.append([])
            stack.append(len(names) - 1)
        elif line.startswith("End Site"):
            parent_idx = stack[-1]
            names.append(f"{names[parent_idx]}_end_site")
            offsets.append(np.zeros(3))
            parents.append(parent_idx)
            channels.append([])
            stack.append(len(names) - 1)
        elif line.startswith("OFFSET"):
            parts = line.split()
            offsets[-1] = np.array([float(parts[1]), float(parts[2]), float(parts[3])]) * scale
        elif line.startswith("CHANNELS"):
            parts = line.split()
            channel_count = int(parts[1])
            channels[-1] = parts[2 : 2 + channel_count]
        elif line == "}":
            if stack:
                stack.pop()
        elif line == "MOTION":
            break

    if not names:
        raise ValueError("No BVH hierarchy found")

    return BVHSkeleton(names, np.asarray(offsets), parents, channels)


def _parse_motion(text: str) -> tuple[list[list[float]], float]:
    motion_start = text.find("MOTION")
    if motion_start < 0:
        raise ValueError("No MOTION section found in BVH file")

    frames: list[list[float]] = []
    frame_time = 1.0 / 30.0
    for raw_line in text[motion_start:].splitlines()[1:]:
        line = raw_line.strip()
        if not line or line.startswith("Frames:"):
            continue
        if line.startswith("Frame Time:"):
            frame_time = float(line.split(":", 1)[1].strip())
            continue
        frames.append([float(value) for value in line.split()])

    if not frames:
        raise ValueError("No BVH motion frames found")
    return frames, frame_time


def _decode_channels(channel_names: list[str], data: list[float], scale: float) -> tuple[Rotation, np.ndarray | None]:
    axes: list[str] = []
    angles: list[float] = []
    position = [None, None, None]

    for channel_name, value in zip(channel_names, data):
        if channel_name == "Xposition":
            position[0] = value
        elif channel_name == "Yposition":
            position[1] = value
        elif channel_name == "Zposition":
            position[2] = value
        elif channel_name == "Xrotation":
            axes.append("X")
            angles.append(value)
        elif channel_name == "Yrotation":
            axes.append("Y")
            angles.append(value)
        elif channel_name == "Zrotation":
            axes.append("Z")
            angles.append(value)

    local_rotation = Rotation.from_euler("".join(axes), angles, degrees=True) if axes else Rotation.identity()
    root_position = np.array(position, dtype=float) * scale if all(value is not None for value in position) else None
    return local_rotation, root_position


def _decode_frames(
    skeleton: BVHSkeleton,
    raw_frames: list[list[float]],
    scale: float,
    cancel_rest_pose: bool,
) -> tuple[list[list[Rotation]], list[np.ndarray]]:
    all_rotations: list[list[Rotation]] = []
    all_root_positions: list[np.ndarray] = []

    expected_channels = sum(len(channel) for channel in skeleton.channels)
    for frame_index, frame_data in enumerate(raw_frames):
        if len(frame_data) != expected_channels:
            raise ValueError(
                f"Frame {frame_index} has {len(frame_data)} values, expected {expected_channels}"
            )

        frame_rotations = [Rotation.identity() for _ in skeleton.names]
        root_position = np.zeros(3)
        cursor = 0
        for joint_index, channel_names in enumerate(skeleton.channels):
            channel_count = len(channel_names)
            if channel_count == 0:
                continue
            local_rotation, maybe_root_position = _decode_channels(
                channel_names,
                frame_data[cursor : cursor + channel_count],
                scale,
            )
            frame_rotations[joint_index] = local_rotation
            if maybe_root_position is not None:
                root_position = maybe_root_position
            cursor += channel_count

        all_rotations.append(frame_rotations)
        all_root_positions.append(root_position)

    if cancel_rest_pose:
        rest_inverse = [rotation.inv() for rotation in all_rotations[0]]
        for frame_rotations in all_rotations:
            for joint_index in range(1, len(frame_rotations)):
                frame_rotations[joint_index] = rest_inverse[joint_index] * frame_rotations[joint_index]

    return all_rotations, all_root_positions


def _rotation_to_wxyz(rotation: Rotation) -> np.ndarray:
    quat_xyzw = rotation.as_quat()
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def _quat_wxyz_to_rotation(quat: np.ndarray) -> Rotation:
    return Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])


def _forward_kinematics(
    skeleton: BVHSkeleton,
    local_rotations: list[list[Rotation]],
    root_positions: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    frame_count = len(local_rotations)
    joint_count = len(skeleton.names)
    global_positions = np.zeros((frame_count, joint_count, 3))
    global_quats = np.zeros((frame_count, joint_count, 4))
    global_quats[:, :, 0] = 1.0

    for frame_index in range(frame_count):
        for joint_index, parent_index in enumerate(skeleton.parents):
            if parent_index == -1:
                global_positions[frame_index, joint_index] = root_positions[frame_index]
                global_quats[frame_index, joint_index] = _rotation_to_wxyz(
                    local_rotations[frame_index][joint_index]
                )
                continue

            parent_rotation = _quat_wxyz_to_rotation(global_quats[frame_index, parent_index])
            global_positions[frame_index, joint_index] = (
                global_positions[frame_index, parent_index]
                + parent_rotation.apply(skeleton.offsets[joint_index])
            )
            global_rotation = parent_rotation * local_rotations[frame_index][joint_index]
            global_quats[frame_index, joint_index] = _rotation_to_wxyz(global_rotation)

    return global_positions, global_quats


def _trim_static_frames(
    global_positions: np.ndarray,
    global_quats: np.ndarray,
    root_positions: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
    first_active = 0
    for frame_index, root_position in enumerate(root_positions):
        if np.linalg.norm(root_position) > 1e-3:
            first_active = frame_index
            break

    if first_active == 0:
        return global_positions, global_quats, root_positions
    return (
        global_positions[first_active:],
        global_quats[first_active:],
        root_positions[first_active:],
    )


def _aliases_for_root(root_name: str) -> tuple[dict[str, str], bool]:
    if root_name == "hip":
        return SNAKE_TO_GMR, False
    if root_name == "Hips":
        return MIXAMO_TO_GMR, True
    raise ValueError(
        f"Unrecognized ForSense BVH root joint name: {root_name!r}. "
        "Expected 'hip' (snake_case export) or 'Hips' (Mixamo export)."
    )


def _build_frames(
    skeleton: BVHSkeleton,
    global_positions: np.ndarray,
    global_quats: np.ndarray,
    aliases: dict[str, str],
) -> list[dict[str, tuple[np.ndarray, np.ndarray]]]:
    frames = []
    for frame_index in range(global_positions.shape[0]):
        frame_data = {}
        for joint_index, joint_name in enumerate(skeleton.names):
            frame_data[joint_name] = (
                global_positions[frame_index, joint_index],
                global_quats[frame_index, joint_index],
            )

        for source_name, target_name in aliases.items():
            if source_name in frame_data:
                frame_data[target_name] = frame_data[source_name]

        if "LeftFoot" in frame_data:
            frame_data["LeftFootMod"] = frame_data["LeftFoot"]
        if "RightFoot" in frame_data:
            frame_data["RightFootMod"] = frame_data["RightFoot"]
        frames.append(frame_data)

    return frames


def _estimate_human_height(frames: list[dict[str, tuple[np.ndarray, np.ndarray]]]) -> float:
    if not frames or "Head" not in frames[0]:
        return 1.75

    if "LeftFootMod" not in frames[0] or "RightFootMod" not in frames[0]:
        return 1.75

    height_samples = []
    for frame_data in frames:
        head_height = frame_data["Head"][0][2]
        foot_height = min(frame_data["LeftFootMod"][0][2], frame_data["RightFootMod"][0][2])
        height_samples.append(head_height - foot_height)
    return max(max(height_samples), 1.0)


def load_forsense_bvh(
    bvh_file: str,
    *,
    scale: float = CM_TO_M,
    trim_static: bool = True,
) -> tuple[list[dict[str, tuple[np.ndarray, np.ndarray]]], float, float]:
    with open(bvh_file, "r", encoding="utf-8") as handle:
        text = handle.read()

    skeleton = _parse_hierarchy(text, scale)
    raw_frames, frame_time = _parse_motion(text)
    aliases, cancel_rest_pose = _aliases_for_root(skeleton.names[0])
    local_rotations, root_positions = _decode_frames(
        skeleton,
        raw_frames,
        scale,
        cancel_rest_pose=cancel_rest_pose,
    )
    global_positions, global_quats = _forward_kinematics(
        skeleton,
        local_rotations,
        root_positions,
    )

    if trim_static:
        global_positions, global_quats, _root_positions = _trim_static_frames(
            global_positions,
            global_quats,
            root_positions,
        )

    frames = _build_frames(skeleton, global_positions, global_quats, aliases)
    return frames, _estimate_human_height(frames), frame_time


def load_mixamo_bvh(bvh_file: str):
    return load_forsense_bvh(bvh_file)
