# IK Config
In our ik config such as `smplx_to_g1.json`, you might find following params. I add annotations here for your understanding.
```json
"ik_match_table1": {
        "pelvis": [ # robot's body name
            "pelvis", # corresponding human body name, here we are using "pelvis" as example
            100, # weight to track 3D positions (xyz)
            10, # weight to track 3D rotations
            [
                0.0, # x offset added to human body "pelvis" x
                0.0, # y offset added to human body "pelvis" y
                0.0 # z offset added to human body "pelvis" z
            ],
            [
                # the rotation (represented as quaternion) applied to human body "pelvis". the order follows scalar first (wxyz)
                0.5,
                -0.5,
                -0.5,
                -0.5
            ]
        ],
      ...
```

---

# BVH Retargeting — ForSense Format

## Overview

BVH files from **ForSense** motion capture hardware go through the following pipeline:

```
ForSense BVH file
      │
      ▼
BVHParser  (parse HIERARCHY + MOTION)
      │   axis_order="xyz", scale=0.01 (cm → m)
      ▼
Anim  (local quats + positions)
      │
      ▼
quat_fk  (forward kinematics → global positions + orientations)
      │
      ▼
FORSENSE_TO_GMR_ALIASES  (rename joints to GMR canonical names)
      │
      ▼
load_bvh_file returns  frames: list[dict[str → (pos, quat)]]
      │
      ▼
GeneralMotionRetargeting.retarget()  (IK solver → robot qpos)
```

---

## Coordinate System

ForSense BVH files are **Z-up** (same as MuJoCo):

| BVH axis | meaning | MuJoCo axis |
|---|---|---|
| X | lateral (left/right) | X |
| Y | anterior (forward/back) | Y |
| Z | vertical (up/down) | Z |

**Critical:** `BVHParser` must be constructed with `axis_order="xyz"` for ForSense files. The xsens loader uses `"zxy"` (different hardware convention) — do **not** reuse that value for ForSense. Using the wrong `axis_order` maps the vertical axis into MuJoCo X, breaking all IK targets and making human height appear as ~0 m.

---

## Root Joint Translation

In the MOTION section of a BVH file, only the **root joint** has 6 channels (3 position + 3 rotation). All other joints have 3 channels (rotation only). `BVHParser._MOTION_data_process` stores the per-frame root translation for any 6-channel node unconditionally — there is no hardcoded name check.

> **Historical bug (fixed):** An earlier version guarded this with `if node.name == "Hips":`, which silently discarded root translation for ForSense files (root named `"hip"`), pinning the pelvis at world origin every frame.

---

## Human Height Scaling

`load_bvh_file` estimates `human_height` as:

```
human_height = max(max(head_Z - min(leftFoot_Z, rightFoot_Z) for each frame), 1.0)
```

This is then used in `GeneralMotionRetargeting` to scale the human skeleton:

```
scale_ratio = actual_human_height / ik_config["human_height_assumption"]  # default assumption: 1.8 m
```

If the axis order is wrong, all Z coordinates appear ~0, height clamps to 1.0 m, and the ratio becomes `1.0 / 1.8 ≈ 0.56` — compressing the entire human skeleton by 44 %.

You can override the detected height with `--human_height` on the CLI.

---

## Calibration / T-Pose Frames

ForSense recordings typically begin with a block of **static calibration frames** where the subject holds a T-pose and all motion values are zero. `load_bvh_file` automatically trims these:

```python
# Trim leading static/calibration frames where root translation is all-zero
first_active = 0
for i in range(anim.pos.shape[0]):
    if np.linalg.norm(anim.pos[i, 0]) > 1e-3:
        first_active = i
        break
```

For example, `20260326_134344.bvh` had **516 leading zero-frames** (5.16 s at 100 FPS) before any actual motion. Without trimming, looped playback would freeze in a T-pose for ~5 s at the start of each cycle.

---

## Joint Naming Conventions

ForSense hardware can export BVH files with two different joint naming schemes. Both are handled by `FORSENSE_TO_GMR_ALIASES` in `general_motion_retargeting/utils/forsense.py`.

### Original ForSense naming (lower_snake_case)

Used by recordings exported directly from ForSense software (e.g. `20260326_134344.bvh`):

| ForSense name | GMR canonical name |
|---|---|
| `hip` | `Hips` (root) |
| `chest` | `Chest4` |
| `head` | `Head` |
| `left_shoulder` / `right_shoulder` | `LeftShoulder` / `RightShoulder` |
| `left_upper_arm` / `right_upper_arm` | `LeftElbow` / `RightElbow` |
| `left_lower_arm` / `right_lower_arm` | `LeftWrist` / `RightWrist` |
| `left_hand` / `right_hand` | `LeftHand` / `RightHand` |
| `left_upper_leg` / `right_upper_leg` | `LeftHip` / `RightHip` |
| `left_lower_leg` / `right_lower_leg` | `LeftKnee` / `RightKnee` |
| `left_foot` / `right_foot` | `LeftFoot` / `RightFoot` |

### Mixamo-compatible naming (CamelCase)

Used by some ForSense exports that follow Mixamo conventions (e.g. `G1-test.bvh`):

| ForSense/Mixamo name | GMR canonical name |
|---|---|
| `Hips` | `Hips` (already correct, no alias needed) |
| `Spine2` | `Chest4` |
| `LeftUpLeg` / `RightUpLeg` | `LeftHip` / `RightHip` |
| `LeftLeg` / `RightLeg` | `LeftKnee` / `RightKnee` |
| `LeftArm` / `RightArm` | `LeftElbow` / `RightElbow` |
| `LeftForeArm` / `RightForeArm` | `LeftWrist` / `RightWrist` |
| `LeftShoulder` / `RightShoulder` | already correct |
| `Head` / `LeftFoot` / `RightFoot` | already correct |

### GMR canonical names used in IK configs

These are the names that appear as keys in each frame dict and must match entries in `bvh_forsense_to_g1.json`:

`Hips`, `Chest4`, `Head`, `LeftShoulder`, `RightShoulder`, `LeftElbow`, `RightElbow`, `LeftWrist`, `RightWrist`, `LeftHip`, `RightHip`, `LeftKnee`, `RightKnee`, `LeftFoot`, `RightFoot`, `LeftFootMod`, `RightFootMod`

`LeftFootMod` and `RightFootMod` are synthetic entries copied from `LeftFoot`/`RightFoot` and used to provide foot orientation targets to the IK solver.

---

## IK Config: `bvh_forsense_to_g1.json`

Located at `general_motion_retargeting/ik_configs/bvh_forsense_to_g1.json`.

- `human_height_assumption`: `1.8` m (used for scale ratio)
- `human_root_name`: `"Hips"`
- `robot_root_name`: `"pelvis"` (Unitree G1 base body)
- `ik_match_table1`: first IK pass — emphasises position for limb roots, high rotation weight for torso/shoulders
- `ik_match_table2`: second IK pass — emphasises position for hips/knees/feet

The two-pass strategy lets the solver first roughly place the skeleton (table1) then refine limb joint angles (table2).

---

## Real-Time Streaming: `scripts/lafan_live_to_ros.py`

Loads a ForSense BVH, retargets each frame to the robot, and publishes three ROS topics at the recorded frame rate:

| Topic | Type | Contents |
|---|---|---|
| `/gmr/joint_states` | `sensor_msgs/JointState` | All non-free joint angles (`qpos[7:]`) |
| `/gmr/base_pose` | `geometry_msgs/PoseStamped` | Root XYZ + quaternion (`qpos[0:7]`) |
| `/gmr/qpos` | `std_msgs/Float64MultiArray` | Full `qpos` vector |

**Usage:**
```bash
python scripts/lafan_live_to_ros.py \
  --bvh_file data/20260326_134344.bvh \
  --format forsense \
  --robot unitree_g1 \
  --ros-version 1 \
  --loop \
  --visualize
```

Key flags:

| Flag | Default | Notes |
|---|---|---|
| `--format` | `lafan1` | Use `forsense` for ForSense hardware |
| `--robot` | `unitree_g1` | Only `unitree_g1` supported for `forsense` format |
| `--loop` | off | Replay BVH continuously |
| `--human_height` | auto-detected | Override height scaling in metres |
| `--motion-fps` | from BVH `Frame Time:` | Override playback rate |
| `--visualize` | off | Open MuJoCo viewer |
| `--solver` | `daqp` | IK solver backend |
| `--damping` | `0.5` | IK solver regularisation |

