#!/usr/bin/env python3
"""Visualize Unitree G1 joint limits in MuJoCo.

The viewer can either sweep one limited hinge joint at a time, or let you
manually nudge the selected joint angle while clamping to its min/max range.
"""

import argparse
import time

import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
from rich import print

from general_motion_retargeting.params import ROBOT_XML_DICT


def limited_hinge_joints(model) -> list[dict]:
    joints = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mj.mjtJoint.mjJNT_HINGE or not model.jnt_limited[joint_id]:
            continue

        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint_id)
        if not name:
            continue

        lower, upper = model.jnt_range[joint_id]
        joints.append(
            {
                "id": joint_id,
                "name": name,
                "qpos_addr": int(model.jnt_qposadr[joint_id]),
                "lower": float(lower),
                "upper": float(upper),
            }
        )
    return joints


def set_neutral_pose(model, data) -> None:
    data.qpos[:] = 0.0
    data.qpos[:3] = [0.0, 0.0, 0.793]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

    for joint in limited_hinge_joints(model):
        lower = joint["lower"]
        upper = joint["upper"]
        if lower <= 0.0 <= upper:
            data.qpos[joint["qpos_addr"]] = 0.0
        else:
            data.qpos[joint["qpos_addr"]] = 0.5 * (lower + upper)


def print_joint_table(joints: list[dict]) -> None:
    print("[bold]G1 limited hinge joints[/bold]")
    for idx, joint in enumerate(joints):
        print(f"{idx:02d} {joint['name']}: [{joint['lower']:.4f}, {joint['upper']:.4f}] rad")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Open G1 in MuJoCo and inspect each limited joint range.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--robot", choices=sorted(ROBOT_XML_DICT.keys()), default="unitree_g1")
    parser.add_argument("--joint", type=str, default=None, help="Start from this joint name or joint index.")
    parser.add_argument("--speed", type=float, default=0.4, help="Sweep speed in cycles per second.")
    parser.add_argument("--sweep", action="store_true", help="Auto-sweep the selected joint through its range.")
    parser.add_argument("--step", type=float, default=0.05, help="Small manual joint-angle step in radians.")
    parser.add_argument("--big-step", type=float, default=0.25, help="Large manual joint-angle step in radians.")
    args = parser.parse_args()

    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT[args.robot]))
    data = mj.MjData(model)
    joints = limited_hinge_joints(model)
    if not joints:
        raise RuntimeError(f"No limited hinge joints found for {args.robot}")

    joint_index = 0
    if args.joint is not None:
        if args.joint.isdigit():
            joint_index = int(args.joint)
        else:
            matching_indices = [i for i, joint in enumerate(joints) if joint["name"] == args.joint]
            if not matching_indices:
                valid_names = ", ".join(joint["name"] for joint in joints)
                raise ValueError(f"Unknown joint {args.joint!r}. Valid joints: {valid_names}")
            joint_index = matching_indices[0]
        if not 0 <= joint_index < len(joints):
            raise ValueError(f"Joint index must be in [0, {len(joints) - 1}], got {joint_index}")

    print(f"Using robot model: {ROBOT_XML_DICT[args.robot]}")
    print_joint_table(joints)
    print(
        "[bold]Keys:[/bold] N next, P previous, [ / ] small angle step, "
        "{ / } large angle step, Space sweep/pause, R reset pose"
    )

    sweep_enabled = args.sweep
    reset_requested = False
    manual_delta = 0.0

    def on_key(keycode: int) -> None:
        nonlocal joint_index, sweep_enabled, reset_requested, manual_delta
        key = chr(keycode).lower() if 0 <= keycode < 256 else ""
        if key == "n":
            joint_index = (joint_index + 1) % len(joints)
        elif key == "p":
            joint_index = (joint_index - 1) % len(joints)
        elif key == " ":
            sweep_enabled = not sweep_enabled
        elif key == "r":
            reset_requested = True
        elif key == "[":
            manual_delta -= args.step
            sweep_enabled = False
        elif key == "]":
            manual_delta += args.step
            sweep_enabled = False
        elif key == "{":
            manual_delta -= args.big_step
            sweep_enabled = False
        elif key == "}":
            manual_delta += args.big_step
            sweep_enabled = False

    set_neutral_pose(model, data)
    mj.mj_forward(model, data)

    with mjv.launch_passive(model, data, key_callback=on_key) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.75]
        viewer.cam.distance = 2.2
        viewer.cam.elevation = -12
        viewer.cam.azimuth = 135

        last_reported_index = None
        start_time = time.monotonic()
        while viewer.is_running():
            joint = joints[joint_index]
            if reset_requested:
                set_neutral_pose(model, data)
                start_time = time.monotonic()
                reset_requested = False

            if joint_index != last_reported_index:
                print(
                    f"Viewing {joint_index:02d} {joint['name']}: "
                    f"[{joint['lower']:.4f}, {joint['upper']:.4f}] rad"
                )
                last_reported_index = joint_index

            if manual_delta != 0.0:
                qpos_addr = joint["qpos_addr"]
                data.qpos[qpos_addr] = np.clip(
                    data.qpos[qpos_addr] + manual_delta,
                    joint["lower"],
                    joint["upper"],
                )
                print(f"{joint['name']} = {data.qpos[qpos_addr]:.4f} rad")
                manual_delta = 0.0

            if sweep_enabled:
                phase = 0.5 + 0.5 * np.sin(2.0 * np.pi * args.speed * (time.monotonic() - start_time))
                data.qpos[joint["qpos_addr"]] = joint["lower"] + phase * (joint["upper"] - joint["lower"])

            mj.mj_forward(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
