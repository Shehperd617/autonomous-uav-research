#!/usr/bin/env python3
"""
Standalone unit test for bbox -> bearing -> range math.
No ROS. No Gazebo. Just numpy. Run anytime to verify the math.
"""
import math
import numpy as np

# Camera intrinsics derived from SDF
HFOV = 1.39626
WIDTH = 640
HEIGHT = 480
fx = (WIDTH / 2) / math.tan(HFOV / 2)
fy = fx
cx = WIDTH / 2
cy = HEIGHT / 2

K = np.array([
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1],
])

# Target physical size in meters (from SDF: 0.4 0.4 0.1)
TARGET_REAL_WIDTH = 0.4

print("Camera intrinsics:")
print(K)
print()


def bbox_to_bearing(x1, y1, x2, y2):
    """Bbox in pixels -> unit bearing vector in camera frame."""
    u = (x1 + x2) / 2.0
    v = (y1 + y2) / 2.0
    x = (u - cx) / fx
    y = (v - cy) / fy
    z = 1.0
    ray = np.array([x, y, z])
    return ray / np.linalg.norm(ray)


def estimate_range(x1, y1, x2, y2):
    """Bbox in pixels -> range estimate in meters."""
    bbox_width_px = abs(x2 - x1)
    if bbox_width_px < 1.0:
        return None
    return fx * TARGET_REAL_WIDTH / bbox_width_px


def target_position_camera_frame(x1, y1, x2, y2):
    """Bbox in pixels -> 3D target position in camera frame (meters)."""
    bearing = bbox_to_bearing(x1, y1, x2, y2)
    rng = estimate_range(x1, y1, x2, y2)
    if rng is None:
        return None
    return bearing * rng


# ============ TESTS ============
print("=" * 60)
print("TEST 1: Target at image center, 50px wide")
print("=" * 60)
bbox = (295, 215, 345, 265)  # 50x50 box centered at (320, 240)
bearing = bbox_to_bearing(*bbox)
rng = estimate_range(*bbox)
pos = target_position_camera_frame(*bbox)
print(f"  bbox = {bbox}")
print(f"  bearing = {bearing}    (should be ~[0, 0, 1])")
print(f"  range = {rng:.2f} m      (target is 0.4m wide, fx={fx:.1f})")
print(f"  position = {pos}")
print(f"  Sanity: pos.z should equal range when bearing is forward.")
print()

print("=" * 60)
print("TEST 2: Target at image center, 100px wide (closer)")
print("=" * 60)
bbox = (270, 190, 370, 290)
rng = estimate_range(*bbox)
print(f"  bbox = {bbox}  (100px wide)")
print(f"  range = {rng:.2f} m   (should be HALF of test 1 since bbox is 2x wider)")
print()

print("=" * 60)
print("TEST 3: Target at image center, 25px wide (further)")
print("=" * 60)
bbox = (307, 227, 333, 253)
rng = estimate_range(*bbox)
print(f"  bbox = {bbox}  (~25px wide)")
print(f"  range = {rng:.2f} m   (should be DOUBLE of test 1)")
print()

print("=" * 60)
print("TEST 4: Target offset to upper-right")
print("=" * 60)
bbox = (500, 100, 550, 150)
bearing = bbox_to_bearing(*bbox)
print(f"  bbox = {bbox}  (center at u=525, v=125)")
print(f"  bearing = {bearing}")
print(f"  Sanity: bearing.x should be POSITIVE (target right of center)")
print(f"          bearing.y should be NEGATIVE (target above center, +y is down)")
print(f"          bearing.z should be POSITIVE (target in front)")
print()

print("=" * 60)
print("TEST 5: Replay best detection from frame 660")
print("=" * 60)
# From spear_yolo_first_detection.png, the bbox looks small and centered
# Eyeballed values - we'll get real ones from the auto-capture later
bbox = (300, 195, 340, 220)  # rough estimate
bearing = bbox_to_bearing(*bbox)
rng = estimate_range(*bbox)
pos = target_position_camera_frame(*bbox)
print(f"  bbox = {bbox}  (approximate from frame660 capture)")
print(f"  bearing = {bearing}")
print(f"  range = {rng:.2f} m")
print(f"  position (camera frame) = {pos}")
print()

print("All tests completed. If all sanity checks make sense, the math is correct.")
