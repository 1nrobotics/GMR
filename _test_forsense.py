"""Quick retargeting smoke-test for forsense format (no ROS required)."""
import numpy as np
from general_motion_retargeting.utils.forsense import load_bvh_file
from general_motion_retargeting import GeneralMotionRetargeting as GMR

print("Loading BVH...")
frames, human_height, _ = load_bvh_file("data/G1-test.bvh")
print(f"  frames={len(frames)}, height={human_height:.3f} m")

print("Initializing retargeter...")
retargeter = GMR(
    src_human="bvh_forsense",
    tgt_robot="unitree_g1",
    actual_human_height=human_height,
    verbose=False,
)

print("Frame 0 keys:", sorted(frames[0].keys()))
print("LeftHip in frame:", "LeftHip" in frames[0])
print("left_upper_leg in frame:", "left_upper_leg" in frames[0])

print("Retargeting first 20 frames...")
errors = []
for i in range(min(20, len(frames))):
    qpos = retargeter.retarget(frames[i])
    errors.append(retargeter.error2())
    if i == 0:
        print(f"  qpos shape={qpos.shape}")
        print(f"  base xyz  ={qpos[:3].round(3)}")
        print(f"  base quat ={qpos[3:7].round(3)}")
        print(f"  joints    ={np.round(qpos[7:], 3)}")

print(f"  IK error (mean over {len(errors)} frames): {np.mean(errors):.4f}")
assert len(frames) > 0, "No frames loaded"
assert human_height > 1.2, f"Height suspiciously low: {human_height}"
assert qpos.shape[0] == retargeter.model.nq, "qpos shape mismatch"
assert np.all(np.isfinite(qpos)), "qpos contains NaN/Inf"
assert qpos[2] > 0.3, f"Robot base Z too low (robot underground?): {qpos[2]:.3f}"
print("PASS")
