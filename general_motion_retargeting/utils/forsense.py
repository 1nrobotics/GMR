import numpy as np

from general_motion_retargeting.utils.xsens_vendor.BVHParser import Anim, BVHParser
import general_motion_retargeting.utils.lafan_vendor.utils as utils


FORSENSE_TO_GMR_ALIASES = {
    # Original ForSense lowercase naming (e.g. 20260326_134344.bvh)
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
    # Mixamo-compatible naming used by some ForSense exports (e.g. G1-test.bvh)
    # Root "Hips" and "LeftShoulder"/"RightShoulder"/"Head"/"LeftFoot"/"RightFoot"
    # are already the correct GMR names so no alias is needed for them.
    "Spine2": "Chest4",
    "LeftUpLeg": "LeftHip",
    "LeftLeg": "LeftKnee",
    "RightUpLeg": "RightHip",
    "RightLeg": "RightKnee",
    "LeftArm": "LeftElbow",
    "LeftForeArm": "LeftWrist",
    "RightArm": "RightElbow",
    "RightForeArm": "RightWrist",
}


def parse_forsense_bvh(
    bvh_file,
    axis_order="xyz",
    scale=0.01,
    start=None,
    end=None,
    reset_to_zero=False,
):
    parser = BVHParser(axis_order=axis_order, scale=scale)
    with open(bvh_file, "r", encoding="utf-8") as handle:
        bvh_text = handle.read()

    rotations, positions = parser.parse(
        bvh_text, start=start, end=end, reset_to_zero=reset_to_zero
    )
    quats, positions, offsets, parents = parser._MOTION_data_post_processing(
        rotations, np.copy(parser.positions), reset_to_zero=reset_to_zero
    )
    anim = Anim(quats, positions, offsets, parents, parser.names)
    global_data = utils.quat_fk(anim.quats, anim.pos, anim.parents)
    return anim, global_data, parser.frame_time


def load_bvh_file(bvh_file, format="forsense"):
    if format != "forsense":
        raise ValueError(f"Invalid format for forsense loader: {format}")

    anim, global_data, frame_time = parse_forsense_bvh(bvh_file)

    # Trim leading static/calibration frames where root translation is all-zero
    first_active = 0
    for i in range(anim.pos.shape[0]):
        if np.linalg.norm(anim.pos[i, 0]) > 1e-3:
            first_active = i
            break
    if first_active > 0:
        anim = Anim(
            anim.quats[first_active:],
            anim.pos[first_active:],
            anim.offsets,
            anim.parents,
            anim.bones,
        )
        global_data = (global_data[0][first_active:], global_data[1][first_active:])

    frames = []
    for frame in range(anim.pos.shape[0]):
        result = {}
        for i, bone in enumerate(anim.bones):
            orientation = global_data[0][frame, i]
            position = global_data[1][frame, i]
            result[bone] = (position, orientation)

        for src_name, target_name in FORSENSE_TO_GMR_ALIASES.items():
            if src_name in result:
                result[target_name] = result[src_name]

        result["LeftFootMod"] = (result["LeftFoot"][0], result["LeftFoot"][1])
        result["RightFootMod"] = (result["RightFoot"][0], result["RightFoot"][1])

        frames.append(result)

    if frames and "Head" in frames[0]:
        height_samples = []
        for frame_data in frames:
            head_height = frame_data["Head"][0][2]
            foot_height = min(
                frame_data["LeftFootMod"][0][2], frame_data["RightFootMod"][0][2]
            )
            height_samples.append(head_height - foot_height)
        human_height = max(max(height_samples), 1.0)
    else:
        human_height = 1.75

    return frames, human_height, frame_time
