#!/usr/bin/env python3
"""Open G1 in MuJoCo with built-in position-control sliders.

This creates a temporary viewer-only XML from the repo's G1 model:
- the floating pelvis joint is removed, keeping the robot upright in world;
- torque motors are replaced by position actuators;
- each actuator ctrlrange is set to the corresponding joint range.

Use MuJoCo's right-side Control panel to move each joint angle manually.
"""

import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import mujoco.viewer as mjv
from rich import print

from general_motion_retargeting.params import ROBOT_XML_DICT


def indent_xml(element: ET.Element, level: int = 0) -> None:
    whitespace = "\n" + level * "  "
    child_whitespace = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_whitespace
        for child in element:
            indent_xml(child, level + 1)
        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = whitespace
    if level and (not element.tail or not element.tail.strip()):
        element.tail = whitespace


def remove_freejoints(element: ET.Element) -> None:
    for child in list(element):
        if child.tag == "freejoint":
            element.remove(child)
        else:
            remove_freejoints(child)


def joint_ranges(root: ET.Element) -> dict[str, tuple[float, float]]:
    ranges = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        range_text = joint.get("range")
        if not name or not range_text:
            continue
        lower, upper = (float(value) for value in range_text.split())
        ranges[name] = (lower, upper)
    return ranges


def build_position_control_xml(source_xml: Path, kp: float, damping: float) -> str:
    tree = ET.parse(source_xml)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is not None:
        meshdir = compiler.get("meshdir")
        if meshdir:
            compiler.set("meshdir", str((source_xml.parent / meshdir).resolve()))

    remove_freejoints(root)

    for joint in root.iter("joint"):
        joint.set("damping", str(damping))

    ranges = joint_ranges(root)
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    actuator.clear()

    for joint_name, (lower, upper) in ranges.items():
        ET.SubElement(
            actuator,
            "position",
            {
                "name": joint_name,
                "joint": joint_name,
                "kp": str(kp),
                "ctrlrange": f"{lower} {upper}",
                "ctrllimited": "true",
            },
        )

    indent_xml(root)
    return ET.tostring(root, encoding="unicode")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Open Unitree G1 in MuJoCo with joint-angle control sliders.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--robot", choices=["unitree_g1", "unitree_g1_with_hands"], default="unitree_g1")
    parser.add_argument("--kp", type=float, default=80.0, help="Position actuator stiffness.")
    parser.add_argument("--damping", type=float, default=2.0, help="Joint damping in the temporary model.")
    args = parser.parse_args()

    source_xml = Path(ROBOT_XML_DICT[args.robot]).resolve()
    xml_text = build_position_control_xml(source_xml, kp=args.kp, damping=args.damping)

    with tempfile.NamedTemporaryFile("w", suffix=".xml", prefix=f"{args.robot}_controls_", delete=False) as handle:
        handle.write(xml_text)
        temp_xml = Path(handle.name)

    print(f"Loaded source model: {source_xml}")
    print(f"Temporary position-control model: {temp_xml}")
    print("Use the MuJoCo right panel: Control -> actuator sliders.")
    print("The sliders are joint-angle targets in radians and are clamped to each joint range.")

    model = mj.MjModel.from_xml_path(str(temp_xml))
    data = mj.MjData(model)
    data.qpos[:] = 0.0
    data.ctrl[:] = 0.0
    mj.mj_forward(model, data)

    mjv.launch(model, data, show_left_ui=True, show_right_ui=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
