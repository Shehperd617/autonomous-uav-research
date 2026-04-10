#!/usr/bin/env python3
"""
Standalone unit test for frame transforms used in Stage 3 perception.

Camera frame (OpenCV convention):
    +X = right (in image)
    +Y = down  (in image)
    +Z = forward (into scene)

Body frame (ROS REP-103 / FRD-rotated FLU):
    +X = forward
    +Y = left
    +Z = up

Camera mount on interceptor (from SDF):
    translation: (0.25, 0, -0.02) m  (forward 0.25, level, 0.02 below center)
    rotation:    identity (no rotational offset, only frame convention)
"""
import numpy as np

# ============ CAMERA -> BODY ROTATION ============
# Pure frame convention rotation. No physical rotation needed because
# the SDF camera pose has zero rotation relative to the body link.
R_CAM_TO_BODY = np.array([
    [ 0,  0,  1],   # x_body = +z_cam
    [-1,  0,  0],   # y_body = -x_cam
    [ 0, -1,  0],   # z_body = -y_cam
], dtype=float)

# Camera mount translation in body frame (meters)
T_CAM_IN_BODY = np.array([0.25, 0.0, -0.02])


def cam_to_body(p_cam):
    """Transform a 3D point from camera frame to body frame."""
    p_cam = np.asarray(p_cam, dtype=float)
    return R_CAM_TO_BODY @ p_cam + T_CAM_IN_BODY


# ============ TESTS ============
print("=" * 60)
print("Camera -> Body frame transform tests")
print("=" * 60)
print(f"\nR_cam_to_body =\n{R_CAM_TO_BODY}")
print(f"T_cam_in_body = {T_CAM_IN_BODY} m\n")

print("-" * 60)
print("TEST 1: Target directly in front of camera, 5m away")
print("-" * 60)
p_cam = np.array([0, 0, 5])
p_body = cam_to_body(p_cam)
print(f"  cam frame:  {p_cam}  (0,0,5 = forward 5m)")
print(f"  body frame: {p_body}")
print(f"  Expected:   [5.25, 0, -0.02]  (5m forward + 0.25 mount offset, "
      f"y=0, z=-0.02 mount drop)")
expected = np.array([5.25, 0, -0.02])
ok = np.allclose(p_body, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}\n")

print("-" * 60)
print("TEST 2: Target 5m forward, 1m above camera")
print("-" * 60)
# In camera frame, 'above' means -Y (because +Y is down in image)
p_cam = np.array([0, -1, 5])
p_body = cam_to_body(p_cam)
print(f"  cam frame:  {p_cam}  (forward 5, image-up 1)")
print(f"  body frame: {p_body}")
print(f"  Expected:   [5.25, 0, 0.98]  (5m fwd + mount, y=0, "
      f"1m up - 0.02 mount drop)")
expected = np.array([5.25, 0, 0.98])
ok = np.allclose(p_body, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}\n")

print("-" * 60)
print("TEST 3: Target 5m forward, 1m to the right of camera")
print("-" * 60)
# In camera frame, 'right' is +X
p_cam = np.array([1, 0, 5])
p_body = cam_to_body(p_cam)
print(f"  cam frame:  {p_cam}  (forward 5, image-right 1)")
print(f"  body frame: {p_body}")
print(f"  Expected:   [5.25, -1, -0.02]  (forward + mount, "
      f"-1 in y because right is -y, mount drop)")
expected = np.array([5.25, -1, -0.02])
ok = np.allclose(p_body, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}\n")

print("-" * 60)
print("TEST 4: Target 5m forward, 1m below camera, 2m to the left")
print("-" * 60)
# In camera frame: forward=+Z, below=+Y (image down), left=-X
p_cam = np.array([-2, 1, 5])
p_body = cam_to_body(p_cam)
print(f"  cam frame:  {p_cam}  (left 2, image-down 1, forward 5)")
print(f"  body frame: {p_body}")
print(f"  Expected:   [5.25, 2, -1.02]  (forward+mount, +y because left, "
      f"-1m down - mount drop)")
expected = np.array([5.25, 2, -1.02])
ok = np.allclose(p_body, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}\n")

print("-" * 60)
print("TEST 5: Replay frame 660 — target estimate at (0, -0.32, 3.80) cam")
print("-" * 60)
# From test_bearing_math.py TEST 5 output:
#   position (camera frame) = [0, -0.32382622, 3.79985127]
p_cam = np.array([0, -0.32382622, 3.79985127])
p_body = cam_to_body(p_cam)
print(f"  cam frame:  {p_cam}")
print(f"  body frame: {p_body}")
print(f"  Interpretation: target is ~{p_body[0]:.2f}m forward, "
      f"{p_body[1]:.2f}m left, {p_body[2]:.2f}m up of body center")
print(f"  Sanity: should be ~4m forward, 0 lateral, slightly above body. "
      f"Target was nearly centered but slightly above horizon.")
print()

print("=" * 60)
print("All transform tests done. If TESTS 1-4 all PASS, frame conventions")
print("are locked and ready for body->world (Step 2).")
print("=" * 60)
