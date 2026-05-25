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
FORSENSE_SNAKE_ALIASES / FORSENSE_MIXAMO_ALIASES
      │   (rename joints to GMR canonical names)
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

ForSense hardware can export BVH files with two different joint naming schemes. Both are handled in `general_motion_retargeting/utils/forsense.py`.

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

`Hips`, `Chest4`, `Head`, `LeftShoulder`, `RightShoulder`, `LeftElbow`, `RightElbow`, `LeftWrist`, `RightWrist`, `LeftHand`, `RightHand`, `LeftHip`, `RightHip`, `LeftKnee`, `RightKnee`, `LeftFoot`, `RightFoot`, `LeftFootMod`, `RightFootMod`

`LeftFootMod` and `RightFootMod` are synthetic entries copied from `LeftFoot`/`RightFoot` and used to provide foot orientation targets to the IK solver.

**Arm naming note:** in the lower_snake_case ForSense export, `LeftElbow`/`RightElbow` are aliases for the `left_upper_arm`/`right_upper_arm` BVH nodes. The Unitree G1 shoulder IK target intentionally tracks these upper-arm segment frames, not the small clavicle-like `LeftShoulder`/`RightShoulder` frames. The elbow target then tracks `LeftWrist`/`RightWrist`, which come from the lower-arm BVH nodes.

---

## IK Config: `bvh_forsense_to_g1.json`

Located at `general_motion_retargeting/ik_configs/bvh_forsense_to_g1.json`.

- `human_height_assumption`: `1.8` m (used for scale ratio)
- `human_root_name`: `"Hips"`
- `robot_root_name`: `"pelvis"` (Unitree G1 base body)
- `ik_match_table1`: first IK pass — emphasises rotation for torso and arm segment orientation
- `ik_match_table2`: second IK pass — emphasises position for pelvis, legs, feet, elbows, and hands
- `collision_avoidance`: optional Mink collision-avoidance settings for arms vs torso/legs; disabled by default in the streaming script unless `--use-collision-avoidance` is passed

The two-pass strategy lets the solver first roughly place the skeleton (table1) then refine limb joint angles (table2).

---

## ForSense → G1 Arm Orientation Offsets

Each entry in `ik_match_table1` and `ik_match_table2` has this layout:

```json
"robot_body": [
  "human_body",
  position_weight,
  rotation_weight,
  position_offset,
  orientation_offset_quat_wxyz
]
```

For example:

```json
"left_shoulder_yaw_link": [
  "LeftElbow",
  0,
  100,
  [0.0, 0.0, 0.0],
  [0.621696694, 0.657317508, -0.159838447, 0.394814622]
]
```

The last value is a quaternion in **scalar-first** order:

```
[w, x, y, z]
```

It is not a robot joint angle. It is a constant frame-conversion rotation from the BVH human segment frame to the MuJoCo G1 body frame.

In `GeneralMotionRetargeting.offset_human_data`, the code applies it as:

```python
target_quat = human_global_quat * orientation_offset
```

Because the offset is right-multiplied, it should be interpreted as a correction in the human body's local coordinate frame. This matters: if the BVH arm frame uses a different local axis convention from G1, the IK target can look correct in position but ask the robot shoulder to twist into a bad orientation.

### Why the arm mapping uses elbow/wrist/hand names

For lower_snake_case ForSense BVH files, the alias table maps:

| BVH node | GMR name | G1 target |
|---|---|---|
| `left_upper_arm` | `LeftElbow` | `left_shoulder_yaw_link` |
| `left_lower_arm` | `LeftWrist` | `left_elbow_link` |
| `left_hand` | `LeftHand` | `left_wrist_yaw_link` |
| `right_upper_arm` | `RightElbow` | `right_shoulder_yaw_link` |
| `right_lower_arm` | `RightWrist` | `right_elbow_link` |
| `right_hand` | `RightHand` | `right_wrist_yaw_link` |

This can look surprising, but it is intentional. The G1 shoulder link should track the human upper-arm segment, not the small `LeftShoulder`/`RightShoulder` clavicle-like BVH node. The G1 elbow link should track the human lower-arm segment, and the G1 wrist link should track the human hand segment.

### How the current ForSense arm offsets were calibrated

The current arm orientation offsets in `bvh_forsense_to_g1.json` were calibrated from:

```
data/bvh_record_0429.bvh
frame 0
```

That frame is a normal standing pose and should correspond to the G1 standing arm pose. The calibration solves:

```text
orientation_offset = inverse(human_frame0_global_orientation) * g1_stand_body_global_orientation
```

For each mapped arm segment:

```text
LeftElbow  -> left_shoulder_yaw_link
LeftWrist  -> left_elbow_link
LeftHand   -> left_wrist_yaw_link
RightElbow -> right_shoulder_yaw_link
RightWrist -> right_elbow_link
RightHand  -> right_wrist_yaw_link
```

The G1 standing arm pose used for calibration is:

```text
left_shoulder_pitch   =  0.2
left_shoulder_roll    =  0.2
left_shoulder_yaw     =  0.0
left_elbow            =  1.28
left_wrist_roll       =  0.0
left_wrist_pitch      =  0.0
left_wrist_yaw        =  0.0

right_shoulder_pitch  =  0.2
right_shoulder_roll   = -0.2
right_shoulder_yaw    =  0.0
right_elbow           =  1.28
right_wrist_roll      =  0.0
right_wrist_pitch     =  0.0
right_wrist_yaw       =  0.0
```

The root heading of the BVH frame is preserved during this calculation. This is important because `bvh_record.bvh` and `bvh_record_0429.bvh` start with different world yaw angles. Calibrating the arm offsets against an identity-world G1 pose can make frame 0 look reasonable in isolation but produce wrong shoulder yaw once the root orientation is included in the IK target.

### Frame-0 verification after calibration

With the current `bvh_record_0429.bvh` offsets, frame 0 gives near-standing G1 shoulder yaw:

```text
left_shoulder_yaw_joint   ≈ 0.49 deg
right_shoulder_yaw_joint  ≈ 4.46 deg
```

Before this calibration, frame 0 pushed the right shoulder into a limit:

```text
left_shoulder_yaw_joint   ≈ -51.92 deg
right_shoulder_yaw_joint  ≈  80.21 deg  (upper limit)
```

### Why offsets alone were not enough

After the frame-0 offset calibration, walking looked good, but later arm motions still caused the left arm to fail. Around frames `1800` to `2310` in `bvh_record_0429.bvh`, the left arm repeatedly selected a bad IK branch:

```text
left_shoulder_roll  -> upper limit
left_shoulder_yaw   -> lower limit
left_elbow          -> sometimes a limit
```

The right arm did not have the same problem. The raw BVH arm positions were reasonably symmetric, but the orientation targets and the G1 joint-limit geometry made the left side easier to pull into the wrong 7-DOF arm branch. This was an IK branch-selection problem, not just a constant offset problem.

---

## Posture Task Regularization

`bvh_forsense_to_g1.json` now includes a `posture_task` block:

```json
"posture_task": {
  "enabled": true,
  "lm_damping": 0.1,
  "use_in_table1": true,
  "use_in_table2": true,
  "target": {
    "left_shoulder_pitch_joint": 0.2,
    "left_shoulder_roll_joint": 0.2,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.28,
    "...": "..."
  },
  "cost": {
    "left_shoulder_pitch_joint": 5.0,
    "left_shoulder_roll_joint": 5.0,
    "left_shoulder_yaw_joint": 5.0,
    "left_elbow_joint": 5.0,
    "...": "..."
  }
}
```

This creates a Mink `PostureTask` in `GeneralMotionRetargeting.setup_posture_task`.

A `PostureTask` is a soft IK regularizer. It does not force the robot to stay in the standing pose. Instead, it says:

> If several joint configurations can satisfy the frame targets, prefer the one closer to this natural G1 arm posture.

This is useful for redundant arms. The G1 arm has enough DOFs that the same hand or wrist target can often be reached through multiple shoulder/elbow branches. Without a posture preference, the solver can choose a mathematically valid but visually wrong branch.

The current cost value is:

```text
5.0 for each left/right shoulder, elbow, and wrist joint
```

This was chosen because it removed the left-arm limit saturation on the full `bvh_record_0429.bvh` sequence while still allowing visible arm motion.

Full-sequence diagnostic result:

```text
Before posture task:
  left arm limit-hit events: 1067
  right arm limit-hit events: 0

After posture task:
  left arm limit-hit events: 0
  right arm limit-hit events: 0
```

At the previously bad frame `1870`, the left arm moved from a saturated branch:

```text
left_shoulder_pitch  ≈ -143.4 deg
left_shoulder_roll   ≈  129.0 deg  (upper limit)
left_shoulder_yaw    ≈  -80.2 deg  (lower limit)
left_elbow           ≈   97.4 deg  (upper limit)
```

to a normal branch:

```text
left_shoulder_pitch  ≈   4.8 deg
left_shoulder_roll   ≈  -4.4 deg
left_shoulder_yaw    ≈  15.9 deg
left_elbow           ≈  67.1 deg
```

If the arm motion becomes too stiff, reduce the posture `cost` values. If the solver again chooses bad shoulder branches, increase them slightly. Good test values are:

```text
2.0  weak bias
5.0  current tested value
10.0 stronger bias, may reduce expressiveness
```

---

## Real-Time Streaming: `scripts/bvh_stream_to_ros.py`

Loads a ForSense BVH, retargets each frame to the robot, and publishes three ROS topics at the recorded frame rate:

| Topic | Type | Contents |
|---|---|---|
| `/gmr/joint_states` | `sensor_msgs/JointState` | All non-free joint angles (`qpos[7:]`) |
| `/gmr/base_pose` | `geometry_msgs/PoseStamped` | Root XYZ + quaternion (`qpos[0:7]`) |
| `/gmr/qpos` | `std_msgs/Float64MultiArray` | Full `qpos` vector |

**Usage:**
```bash
python scripts/bvh_stream_to_ros.py \
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
| `--use-velocity-limit` | off | Add a velocity limit to the IK solve |
| `--use-collision-avoidance` | off | Enable optional collision avoidance from the IK config |
| `--arm-out-clamp` | `0.0` | Clamp the minimum outward G1 shoulder-roll angle in radians after IK |

### Arm Clearance Options

If the retargeted G1 hands pass through the torso or legs, start with the simple shoulder-roll clamp:

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

`--arm-out-clamp` is applied after IK. For G1, `0.15` clamps `left_shoulder_roll_joint` to at least `+0.15` radians and `right_shoulder_roll_joint` to at most `-0.15` radians, keeping both arms from moving too far inward. A practical starting range is `0.10` to `0.20` radians.

For a constraint-based approach, pass `--use-collision-avoidance`. This enables the `collision_avoidance` block in `bvh_forsense_to_g1.json`, currently configured for elbow/wrist vs torso and wrist vs same-side hip/knee links. This is more physically aware than a shoulder bias, but it can make the IK solve stiffer and more expensive.
