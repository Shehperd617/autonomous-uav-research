#!/usr/bin/env python3
"""
Standalone unit test for body -> world frame transform.

Takes a target position in the interceptor's body frame and the
interceptor's pose in the world (position + quaternion), and produces
the target's position in world frame.

World frame = ROS REP-103 ENU/FLU global:
    +X = east (or world forward)
    +Y = north (or world left)
    +Z = up

Body frame = same convention, but rotated and translated by the
interceptor's current pose.
"""
import numpy as np


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    """
    Convert a quaternion (w, x, y, z) to a 3x3 rotation matrix.
    Standard formula from Hamilton convention.
    """
    # Normalize first to be safe
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-10:
        return np.eye(3)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n

    R = np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])
    return R


def body_to_world(p_body, interceptor_pos, interceptor_quat):
    """
    Transform a 3D point from body frame to world frame.

    p_body:           (3,) target position in body frame
    interceptor_pos:  (3,) interceptor position in world frame
    interceptor_quat: (4,) interceptor orientation (qw, qx, qy, qz)
    """
    p_body = np.asarray(p_body, dtype=float)
    t_world = np.asarray(interceptor_pos, dtype=float)
    qw, qx, qy, qz = interceptor_quat
    R = quaternion_to_rotation_matrix(qw, qx, qy, qz)
    return R @ p_body + t_world


# ============ TESTS ============
print("=" * 60)
print("Body -> World frame transform tests")
print("=" * 60)

print("\n" + "-" * 60)
print("TEST 1: Identity orientation, interceptor at origin")
print("-" * 60)
# Interceptor at world origin, no rotation, target 5m forward in body
p_body = np.array([5, 0, 0])
int_pos = np.array([0, 0, 0])
int_quat = np.array([1, 0, 0, 0])  # identity quaternion (w=1, x=y=z=0)
p_world = body_to_world(p_body, int_pos, int_quat)
print(f"  body:  {p_body}  (5m forward in body)")
print(f"  int:   pos={int_pos}, quat={int_quat} (identity)")
print(f"  world: {p_world}")
print(f"  Expected: [5, 0, 0]  (no rotation, no translation)")
expected = np.array([5, 0, 0])
ok = np.allclose(p_world, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}")

print("\n" + "-" * 60)
print("TEST 2: Interceptor translated to (10, 20, 30), no rotation")
print("-" * 60)
p_body = np.array([5, 0, 0])
int_pos = np.array([10, 20, 30])
int_quat = np.array([1, 0, 0, 0])
p_world = body_to_world(p_body, int_pos, int_quat)
print(f"  body:  {p_body}")
print(f"  int:   pos={int_pos}")
print(f"  world: {p_world}")
print(f"  Expected: [15, 20, 30]  (5m forward of (10,20,30))")
expected = np.array([15, 20, 30])
ok = np.allclose(p_world, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}")

print("\n" + "-" * 60)
print("TEST 3: Interceptor at origin, yawed 90 deg left (facing +Y)")
print("-" * 60)
# Yaw 90 deg counterclockwise (left): rotation about +Z axis by +pi/2
# Quaternion for rotation pi/2 about Z:
#   qw = cos(pi/4) = 0.7071
#   qz = sin(pi/4) = 0.7071
p_body = np.array([5, 0, 0])  # 5m forward in body
int_pos = np.array([0, 0, 0])
int_quat = np.array([0.70710678, 0, 0, 0.70710678])
p_world = body_to_world(p_body, int_pos, int_quat)
print(f"  body:  {p_body}  (5m forward in body)")
print(f"  int:   yawed 90 deg left, facing world +Y")
print(f"  world: {p_world}")
print(f"  Expected: [0, 5, 0]  (forward in body = +Y in world after 90 deg yaw)")
expected = np.array([0, 5, 0])
ok = np.allclose(p_world, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}")

print("\n" + "-" * 60)
print("TEST 4: Interceptor yawed 180 deg (facing -X)")
print("-" * 60)
# Yaw 180 deg: qw = cos(pi/2) = 0, qz = sin(pi/2) = 1
p_body = np.array([5, 0, 0])
int_pos = np.array([0, 0, 0])
int_quat = np.array([0, 0, 0, 1])
p_world = body_to_world(p_body, int_pos, int_quat)
print(f"  body:  {p_body}  (5m forward in body)")
print(f"  int:   yawed 180 deg, facing world -X")
print(f"  world: {p_world}")
print(f"  Expected: [-5, 0, 0]  (forward in body = -X in world)")
expected = np.array([-5, 0, 0])
ok = np.allclose(p_world, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}")

print("\n" + "-" * 60)
print("TEST 5: Combined - interceptor at (10, 20, 30) yawed 90 deg")
print("-" * 60)
p_body = np.array([5, 2, 1])  # 5m forward, 2m left, 1m up in body
int_pos = np.array([10, 20, 30])
int_quat = np.array([0.70710678, 0, 0, 0.70710678])  # yaw 90 deg left
p_world = body_to_world(p_body, int_pos, int_quat)
print(f"  body:  {p_body}  (5 fwd, 2 left, 1 up)")
print(f"  int:   pos={int_pos}, yawed 90 deg left")
print(f"  world: {p_world}")
print(f"  Expected: [10 - 2, 20 + 5, 30 + 1] = [8, 25, 31]")
print(f"    (yawed: body-forward becomes +Y world, body-left becomes -X world,")
print(f"     body-up stays +Z world; then add interceptor position)")
expected = np.array([8, 25, 31])
ok = np.allclose(p_world, expected, atol=1e-6)
print(f"  {'PASS' if ok else 'FAIL'}")

print("\n" + "-" * 60)
print("TEST 6: Replay frame 660 with realistic interceptor pose")
print("-" * 60)
# From frame 660 capture, in body frame we computed: [4.05, 0.00, 0.30]
# Suppose at the moment of that frame, the interceptor was at world
# position (50, 30, 15) facing world +X (identity yaw)
p_body = np.array([4.04985127, 0.0, 0.30382622])
int_pos = np.array([50, 30, 15])
int_quat = np.array([1, 0, 0, 0])
p_world = body_to_world(p_body, int_pos, int_quat)
print(f"  body:  {p_body}  (target localized in body frame)")
print(f"  int:   at world {int_pos}, facing +X")
print(f"  world: {p_world}")
print(f"  Interpretation: target is at world position {p_world}")
print(f"  Sanity: ~4m forward of interceptor along +X, slight altitude bump.")

print("\n" + "=" * 60)
print("All body->world tests done. If TESTS 1-5 all PASS, the rotation")
print("math is correct and we can plug in real interceptor poses from ROS.")
print("=" * 60)
