#!/usr/bin/env python3
"""
SPEAR Gazebo Bridge v2 — Fast continuous velocity control
Uses persistent subprocess pipes instead of spawning per-command.

Terminal 1:  ign gazebo two_drone_world.sdf    (press Play)
Terminal 2:  python3 spear_gazebo_bridge.py
"""

import numpy as np
import subprocess
import threading
import signal
import time
import sys
import re


# ─── Config ──────────────────────────────────────────────────────────────────

WAYPOINTS = [
    # Phase 1: Straight approach
    (50, 20, 20),
    (30, 10, 18),
    (10, 5, 16),
    # Phase 2: Evasive
    (0, 20, 18),
    (-10, -5, 20),
    (-20, 15, 15),
    # Phase 3: Aggressive
    (-15, 5, 10),
    (-5, 10, 22),
    (10, 15, 17),
]

TARGET_SPEED = 5.0
TARGET_SPEED_FAST = 8.0
WP_THRESHOLD = 4.0
INTERCEPT_DIST = 2.5
PURSUIT_SPEED = 12.0
CONTROL_HZ = 10  # 10 Hz is plenty for ign topic approach


# ─── Velocity Publisher ──────────────────────────────────────────────────────

def publish_vel(topic, lx, ly, lz):
    """Publish velocity — fire and forget."""
    msg = f'linear: {{x: {lx:.3f}, y: {ly:.3f}, z: {lz:.3f}}}'
    try:
        subprocess.Popen(
            ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Twist', '-p', msg],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def get_poses():
    """Get interceptor and target positions. Returns (int_pos, tgt_pos) or Nones."""
    try:
        result = subprocess.run(
            ['ign', 'topic', '-t', '/world/spear_intercept/dynamic_pose/info',
             '-e', '-n', '1'],
            timeout=3.0, capture_output=True, text=True
        )
        text = result.stdout

        int_pos = _parse_model_pos(text, "interceptor")
        tgt_pos = _parse_model_pos(text, "target")
        return int_pos, tgt_pos

    except Exception as e:
        return None, None


def _parse_model_pos(text, model_name):
    """Parse position for a model from pose info output."""
    # Find the model name, then find the next position block
    pattern = rf'name: "{model_name}".*?position\s*\{{(.*?)\}}'
    match = re.search(pattern, text, re.DOTALL)
    if not match:
        return None

    pos_block = match.group(1)
    x = _extract_val(pos_block, 'x')
    y = _extract_val(pos_block, 'y')
    z = _extract_val(pos_block, 'z')
    return np.array([x, y, z])


def _extract_val(text, key):
    """Extract a numeric value for a key from text."""
    match = re.search(rf'{key}:\s*([-\d.e+]+)', text)
    return float(match.group(1)) if match else 0.0


# ─── Main Loop ───────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("  SPEAR Gazebo Bridge v2")
    print("=" * 55)
    print()
    print("  Make sure Gazebo is running with Play pressed.")
    print("  Ctrl+C to stop.")
    print()

    # Wait for Gazebo
    print("  Connecting...", end='', flush=True)
    for _ in range(20):
        int_pos, tgt_pos = get_poses()
        if int_pos is not None:
            print(f" OK!")
            print(f"  Interceptor: {int_pos}")
            print(f"  Target:      {tgt_pos}")
            break
        print(".", end='', flush=True)
        time.sleep(1)
    else:
        print("\n  Could not get poses. Is Gazebo running with Play pressed?")
        return

    # State
    wp_idx = 0
    intercepts = 0
    t_start = time.time()
    dt = 1.0 / CONTROL_HZ

    # Graceful exit
    running = [True]
    def handler(s, f):
        running[0] = False
    signal.signal(signal.SIGINT, handler)

    print(f"\n  Running at {CONTROL_HZ} Hz. Target has {len(WAYPOINTS)} waypoints.\n")

    loop = 0
    while running[0]:
        t0 = time.time()
        elapsed = t0 - t_start

        # Get poses (every loop since we're only at 10 Hz)
        int_pos, tgt_pos = get_poses()
        if int_pos is None or tgt_pos is None:
            time.sleep(dt)
            continue

        # ── Target: fly waypoints ──
        if wp_idx < len(WAYPOINTS):
            wp = np.array(WAYPOINTS[wp_idx])
            diff = wp - tgt_pos
            dist_to_wp = np.linalg.norm(diff)

            if dist_to_wp < WP_THRESHOLD:
                wp_idx += 1
                phase = 1 if wp_idx < 3 else (2 if wp_idx < 6 else 3)
                print(f"  [TARGET] Waypoint {wp_idx}/{len(WAYPOINTS)} | Phase {phase}")
                if wp_idx >= len(WAYPOINTS):
                    wp_idx = 0  # Loop
                continue

            speed = TARGET_SPEED if wp_idx < 3 else TARGET_SPEED_FAST
            tgt_vel = (diff / dist_to_wp) * speed
        else:
            wp_idx = 0
            tgt_vel = np.zeros(3)

        publish_vel("/model/target/cmd_vel", tgt_vel[0], tgt_vel[1], tgt_vel[2])

        # ── Interceptor: simple pursuit (replace with SPEAR later) ──
        chase = tgt_pos - int_pos
        chase_dist = np.linalg.norm(chase)
        if chase_dist > 0.5:
            int_vel = (chase / chase_dist) * PURSUIT_SPEED
        else:
            int_vel = np.zeros(3)

        publish_vel("/model/interceptor/cmd_vel", int_vel[0], int_vel[1], int_vel[2])

        # ── Check intercept ──
        separation = np.linalg.norm(tgt_pos - int_pos)
        if separation < INTERCEPT_DIST:
            intercepts += 1
            print(f"\n  *** INTERCEPT #{intercepts} at t={elapsed:.1f}s "
                  f"| sep={separation:.1f}m ***\n")

        # ── Status ──
        if loop % 5 == 0:  # Every 0.5s
            print(f"  t={elapsed:5.1f}s | "
                  f"Tgt({tgt_pos[0]:6.1f},{tgt_pos[1]:6.1f},{tgt_pos[2]:6.1f}) | "
                  f"Int({int_pos[0]:6.1f},{int_pos[1]:6.1f},{int_pos[2]:6.1f}) | "
                  f"Sep:{separation:5.1f}m | WP:{wp_idx}/{len(WAYPOINTS)}")

        loop += 1

        # Rate limit
        sleep = dt - (time.time() - t0)
        if sleep > 0:
            time.sleep(sleep)

    # Cleanup
    print(f"\n  Stopped. Intercepts: {intercepts} | Runtime: {time.time()-t_start:.1f}s")
    publish_vel("/model/target/cmd_vel", 0, 0, 0)
    publish_vel("/model/interceptor/cmd_vel", 0, 0, 0)


if __name__ == "__main__":
    main()
