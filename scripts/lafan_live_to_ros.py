#!/usr/bin/env python3
"""
Replay a LAFAN/Nokov BVH file in real time, retarget it to a robot, and publish
the result to ROS topics.

Publishes:
- sensor_msgs/JointState on the joint topic
- geometry_msgs/PoseStamped on the base pose topic
- std_msgs/Float64MultiArray on the full qpos topic
"""

import argparse
import pathlib
import signal
import sys
import time

import numpy as np
from rich import print

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_bvh_file


g_running = True


def signal_handler(signum, frame):
    del frame
    global g_running
    print(f"\nReceived signal {signum}, shutting down...")
    g_running = False


def resolve_input_path(path_str: str) -> pathlib.Path:
    here = pathlib.Path(__file__).resolve().parent
    repo_root = here.parent

    path = pathlib.Path(path_str)
    if path.is_absolute():
        return path
    if path.exists():
        return path.resolve()

    repo_relative_path = repo_root / path
    if repo_relative_path.exists():
        return repo_relative_path.resolve()

    return path


def read_bvh_frame_time(bvh_path: pathlib.Path) -> float | None:
    with open(bvh_path, "r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped.startswith("Frame Time:"):
                return float(stripped.split(":", 1)[1].strip())
    return None


def build_joint_names(model) -> list[str]:
    import mujoco as mj

    joint_names = []
    for joint_id in range(model.njnt):
        joint_type = model.jnt_type[joint_id]
        joint_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint_id)

        if joint_type == mj.mjtJoint.mjJNT_FREE:
            continue

        qpos_width = 4 if joint_type == mj.mjtJoint.mjJNT_BALL else 1
        if qpos_width == 1:
            joint_names.append(joint_name)
        else:
            for idx in range(qpos_width):
                joint_names.append(f"{joint_name}_{idx}")

    return joint_names


class RosBridge:
    def __init__(
        self,
        ros_version: str,
        node_name: str,
        joint_topic: str,
        base_pose_topic: str,
        qpos_topic: str,
        frame_id: str,
        joint_names: list[str],
    ) -> None:
        self.ros_version = self._detect_ros_version(ros_version)
        self.frame_id = frame_id
        self.joint_names = joint_names

        if self.ros_version == "1":
            import rospy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray

            self.rospy = rospy
            self.PoseStamped = PoseStamped
            self.JointState = JointState
            self.Float64MultiArray = Float64MultiArray

            if not rospy.core.is_initialized():
                rospy.init_node(node_name, anonymous=True)

            self.joint_pub = rospy.Publisher(joint_topic, JointState, queue_size=10)
            self.base_pose_pub = rospy.Publisher(base_pose_topic, PoseStamped, queue_size=10)
            self.qpos_pub = rospy.Publisher(qpos_topic, Float64MultiArray, queue_size=10)

        elif self.ros_version == "2":
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray

            self.rclpy = rclpy
            self.PoseStamped = PoseStamped
            self.JointState = JointState
            self.Float64MultiArray = Float64MultiArray
            self._owns_ros2_context = not rclpy.ok()
            if self._owns_ros2_context:
                rclpy.init(args=None)
            self.node = Node(node_name)
            self.joint_pub = self.node.create_publisher(JointState, joint_topic, 10)
            self.base_pose_pub = self.node.create_publisher(PoseStamped, base_pose_topic, 10)
            self.qpos_pub = self.node.create_publisher(Float64MultiArray, qpos_topic, 10)
        else:
            raise ValueError(f"Unsupported ROS version: {self.ros_version}")

    def _detect_ros_version(self, requested_version: str) -> str:
        if requested_version in {"1", "2"}:
            return requested_version

        try:
            import rospy  # noqa: F401

            return "1"
        except ImportError:
            pass

        try:
            import rclpy  # noqa: F401

            return "2"
        except ImportError as exc:
            raise ImportError(
                "Could not import ROS 1 or ROS 2 Python packages. "
                "Install rospy or rclpy and the standard message packages, "
                "or pass --ros-version 1/2 after sourcing your ROS environment."
            ) from exc

    def _ros1_stamp(self):
        return self.rospy.Time.now()

    def _ros2_stamp(self):
        return self.node.get_clock().now().to_msg()

    def publish(self, qpos: np.ndarray) -> None:
        stamp = self._ros1_stamp() if self.ros_version == "1" else self._ros2_stamp()

        joint_msg = self.JointState()
        joint_msg.header.stamp = stamp
        joint_msg.header.frame_id = self.frame_id
        joint_msg.name = self.joint_names
        joint_msg.position = qpos[7:].tolist()
        self.joint_pub.publish(joint_msg)

        pose_msg = self.PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(qpos[0])
        pose_msg.pose.position.y = float(qpos[1])
        pose_msg.pose.position.z = float(qpos[2])
        pose_msg.pose.orientation.w = float(qpos[3])
        pose_msg.pose.orientation.x = float(qpos[4])
        pose_msg.pose.orientation.y = float(qpos[5])
        pose_msg.pose.orientation.z = float(qpos[6])
        self.base_pose_pub.publish(pose_msg)

        qpos_msg = self.Float64MultiArray()
        qpos_msg.data = qpos.tolist()
        self.qpos_pub.publish(qpos_msg)

        if self.ros_version == "2":
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def ok(self) -> bool:
        if self.ros_version == "1":
            return not self.rospy.is_shutdown()
        return self.rclpy.ok()

    def close(self) -> None:
        if self.ros_version == "2":
            self.node.destroy_node()
            if self._owns_ros2_context and self.rclpy.ok():
                self.rclpy.shutdown()


def main() -> int:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    parser = argparse.ArgumentParser(
        description="Realtime LAFAN BVH retargeting and ROS topic streaming",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--bvh_file", required=True, type=str, help="BVH motion file to stream.")
    parser.add_argument("--format", choices=["lafan1", "nokov"], default="lafan1")
    parser.add_argument(
        "--robot",
        choices=[
            "unitree_g1",
            "unitree_g1_with_hands",
            "booster_t1_29dof",
            "fourier_n1",
            "engineai_pm01",
            "pal_talos",
        ],
        default="unitree_g1",
    )
    parser.add_argument(
        "--ros-version",
        choices=["auto", "1", "2"],
        default="auto",
        help="Auto-detect ROS 1 vs ROS 2 unless a specific version is required.",
    )
    parser.add_argument("--node-name", default="lafan_live_to_ros")
    parser.add_argument("--joint-topic", default="/gmr/joint_states")
    parser.add_argument("--base-pose-topic", default="/gmr/base_pose")
    parser.add_argument("--qpos-topic", default="/gmr/qpos")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument(
        "--loop",
        action="store_true",
        default=False,
        help="Loop the BVH motion continuously.",
    )
    parser.add_argument(
        "--motion-fps",
        type=float,
        default=None,
        help="Override BVH frame rate. If omitted, uses BVH Frame Time when available.",
    )
    parser.add_argument(
        "--human_height",
        type=float,
        default=None,
        help="Optional actual human height in meters for retarget scaling.",
    )
    parser.add_argument("--visualize", action="store_true", default=False)
    parser.add_argument("--follow-camera", dest="follow_camera", action="store_true", default=True)
    parser.add_argument("--no-follow-camera", dest="follow_camera", action="store_false")
    parser.add_argument("--record_video", action="store_true", default=False)
    parser.add_argument("--video_path", type=str, default="videos/lafan_live_to_ros.mp4")
    parser.add_argument("--solver", type=str, default="daqp")
    parser.add_argument("--damping", type=float, default=0.5)
    parser.add_argument("--use-velocity-limit", action="store_true", default=False)

    args = parser.parse_args()

    bvh_path = resolve_input_path(args.bvh_file)
    if not bvh_path.exists():
        raise FileNotFoundError(f"BVH file not found: {bvh_path}")

    frame_time = read_bvh_frame_time(bvh_path)
    motion_fps = args.motion_fps if args.motion_fps is not None else (1.0 / frame_time if frame_time else 30.0)
    frame_period = 1.0 / motion_fps

    print(f"Loading BVH: {bvh_path}")
    human_frames, actual_human_height = load_bvh_file(str(bvh_path), format=args.format)
    if args.human_height is not None:
        actual_human_height = args.human_height

    print("Initializing retargeter...")
    retargeter = GMR(
        src_human=f"bvh_{args.format}",
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
        solver=args.solver,
        damping=args.damping,
        use_velocity_limit=args.use_velocity_limit,
    )

    joint_names = build_joint_names(retargeter.model)
    if len(joint_names) != retargeter.model.nq - 7:
        raise ValueError(
            f"Joint name count ({len(joint_names)}) does not match robot DoF count ({retargeter.model.nq - 7})."
        )

    ros_bridge = RosBridge(
        ros_version=args.ros_version,
        node_name=args.node_name,
        joint_topic=args.joint_topic,
        base_pose_topic=args.base_pose_topic,
        qpos_topic=args.qpos_topic,
        frame_id=args.frame_id,
        joint_names=joint_names,
    )

    viewer = None
    if args.visualize:
        viewer = RobotMotionViewer(
            robot_type=args.robot,
            motion_fps=motion_fps,
            transparent_robot=0,
            record_video=args.record_video,
            video_path=args.video_path,
        )

    print(f"Loaded {len(human_frames)} frames at {motion_fps:.3f} FPS")
    print(f"Publishing ROS topics: {args.joint_topic}, {args.base_pose_topic}, {args.qpos_topic}")
    print("Starting realtime retargeting... Press Ctrl+C to stop.")

    frame_idx = 0
    total_frames = 0
    fps_counter = 0
    fps_window_start = time.monotonic()
    next_tick = time.monotonic()

    try:
        while g_running and ros_bridge.ok():
            human_frame = human_frames[frame_idx]
            qpos = retargeter.retarget(human_frame)
            ros_bridge.publish(qpos)

            if viewer is not None:
                viewer.step(
                    root_pos=qpos[:3],
                    root_rot=qpos[3:7],
                    dof_pos=qpos[7:],
                    human_motion_data=retargeter.scaled_human_data,
                    rate_limit=False,
                    follow_camera=args.follow_camera,
                )

            total_frames += 1
            fps_counter += 1
            now = time.monotonic()
            if now - fps_window_start >= 2.0:
                print(f"Streaming FPS: {fps_counter / (now - fps_window_start):.2f} | Published: {total_frames}")
                fps_counter = 0
                fps_window_start = now

            frame_idx += 1
            if frame_idx >= len(human_frames):
                if not args.loop:
                    break
                frame_idx = 0

            next_tick += frame_period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    finally:
        if viewer is not None:
            viewer.close()
        ros_bridge.close()
        print(f"Stopped after publishing {total_frames} frames.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
