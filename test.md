# ForSense BVH to Fourier N1

## Offline retargeting preview

Use this path when you want to load a ForSense BVH file, retarget it to Fourier N1, and view the result in MuJoCo:

```bash
python scripts/bvh_to_robot.py \
  --bvh_file /path/to/forsense_motion.bvh \
  --format forsense \
  --robot fourier_n1 \
  --rate_limit
```

To save the retargeted robot motion:

```bash
python scripts/bvh_to_robot.py \
  --bvh_file /path/to/forsense_motion.bvh \
  --format forsense \
  --robot fourier_n1 \
  --save_path retargeting_data/fourier_n1_forsense.pkl
```

## ROS streaming

Source your ROS environment first, then run the streamer:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file /path/to/forsense_motion.bvh \
  --format forsense \
  --robot fourier_n1 \
  --ros-version auto
```

The streamer publishes:

- `sensor_msgs/JointState` on `/gmr/joint_states`
- `geometry_msgs/PoseStamped` on `/gmr/base_pose`
- `std_msgs/Float64MultiArray` on `/gmr/qpos`

Topic names can be overridden:

```bash
python scripts/bvh_stream_to_ros.py \
  --bvh_file /path/to/forsense_motion.bvh \
  --format forsense \
  --robot fourier_n1 \
  --joint-topic /fourier_n1/joint_states \
  --base-pose-topic /fourier_n1/base_pose \
  --qpos-topic /fourier_n1/qpos
```

Add `--visualize` to show the MuJoCo viewer while publishing, and add `--loop` to replay the BVH continuously.
