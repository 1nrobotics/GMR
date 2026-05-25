# Test Notes

## Test `bvh_to_robot.py`

Use this command to retarget a LAFAN BVH file offline, visualize it in MuJoCo, and save the retargeted motion to a `.pkl` file:

```bash
python scripts/bvh_to_robot.py \
  --bvh_file data/dance1_subject1.bvh \
  --robot unitree_g1 \
  --save_path data/dance1_subject1.pkl \
  --format lafan1
```

Add `--rate_limit` if you want playback to run in real time.

## Complete ROS Setup For `bvh_stream_to_ros.py`

`scripts/bvh_stream_to_ros.py` needs a working ROS Python environment before it can publish topics.

### ROS Noetic With Conda

If you are using ROS Noetic and running Python from a conda environment, use this setup in the same shell before launching the script:

```bash
conda activate gmr
source /opt/ros/noetic/setup.bash
export PYTHONPATH=$CONDA_PREFIX/lib/python3.10/site-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog
```

Why this is needed:

- ROS Noetic installs `rospy` and ROS message packages into Ubuntu system paths, not into your conda env.
- Your conda env still needs to see those ROS packages, so we add ROS and system `dist-packages` to `PYTHONPATH`.
- The order matters: conda `site-packages` must stay first, otherwise Python may import Ubuntu's old `numpy` instead of the conda one and crash.
- `ROS_LOG_DIR=/tmp/roslog` avoids permission problems when ROS tries to write logs.

Verify the environment:

```bash
python - <<'PY'
import numpy
import yaml
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
print("ROS Noetic + conda OK")
print("numpy:", numpy.__version__, numpy.__file__)
PY
```

### ROS 1 Noetic

If you are using ROS 1 Noetic, make sure these are available:

```bash
sudo apt update
sudo apt install -y python3-yaml python3-rospy ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-std-msgs
source /opt/ros/noetic/setup.bash
```

Check that Python can import the required modules:

```bash
python - <<'PY'
import yaml
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
print("ROS 1 Python setup OK")
PY
```

### ROS 2

If you are using ROS 2, source your ROS 2 environment first, then make sure `rclpy` and the standard message packages are available in the same Python environment:

```bash
source /opt/ros/<your_ros2_distro>/setup.bash
python - <<'PY'
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
print("ROS 2 Python setup OK")
PY
```

## Run `bvh_stream_to_ros.py`

After ROS is set up, run:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file data/dance1_subject1.bvh \
  --format lafan1 \
  --robot unitree_g1 \
  --ros-version 1 \
  --loop \
  --visualize
```

For a ForSense BVH recording on Unitree G1:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file data/bvh_record.bvh \
  --format forsense \
  --robot unitree_g1 \
  --ros-version 1 \
  --loop \
  --visualize \
  --follow-camera
```

Expected result:

- Leg and torso motion should follow the BVH.
- Arm motion should come from the upper-arm and lower-arm ForSense segments, not the clavicle-like shoulder nodes.
- ROS publishes `/gmr/joint_states`, `/gmr/base_pose`, and `/gmr/qpos`.

### Arm Clearance Checks

If the hands pass through the torso or legs, first try the lightweight shoulder-roll clamp:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file data/bvh_record.bvh \
  --format forsense \
  --robot unitree_g1 \
  --ros-version 1 \
  --loop \
  --visualize \
  --follow-camera \
  --arm-out-clamp 0.15
```

Use `0.10` to `0.20` radians as the usual test range. For G1, `0.15` clamps the left shoulder roll to at least `+0.15` radians and the right shoulder roll to at most `-0.15` radians.

For constraint-based collision avoidance, run:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file data/bvh_record.bvh \
  --format forsense \
  --robot unitree_g1 \
  --ros-version 1 \
  --loop \
  --visualize \
  --follow-camera \
  --use-collision-avoidance
```

Expected result:

- Default behavior has collision avoidance disabled.
- With `--use-collision-avoidance`, the retargeter should include `CollisionAvoidanceLimit`.
- If avoidance reacts too late, increase `detection_distance` in `general_motion_retargeting/ik_configs/bvh_forsense_to_g1.json`.
- If hands still get too close, increase `min_distance` in the same config.

### MuJoCo Debug Visualization

For debugging, keep `--visualize` enabled so you can see the retargeted robot motion live in MuJoCo while the ROS topics are being published.

Debug command:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file data/dance1_subject1.bvh \
  --format lafan1 \
  --robot unitree_g1 \
  --ros-version 1 \
  --loop \
  --visualize \
  --follow-camera
```

Why this helps:

- You can immediately see whether the retargeted pose looks reasonable before checking downstream ROS consumers.
- It helps separate retargeting problems from ROS transport problems.
- If the topics are being published but the robot pose looks wrong in MuJoCo, the issue is likely in retargeting or input data rather than ROS messaging.

This publishes:

- `sensor_msgs/JointState` on `/gmr/joint_states`
- `geometry_msgs/PoseStamped` on `/gmr/base_pose`
- `std_msgs/Float64MultiArray` on `/gmr/qpos`

## Difference From `bvh_to_robot.py`

- `bvh_to_robot.py` is mainly for offline retargeting, MuJoCo visualization, and optional `.pkl` export.
- `bvh_stream_to_ros.py` is mainly for real-time retargeting and ROS topic publishing.
- `bvh_stream_to_ros.py` can also visualize, but ROS output is its main job.
