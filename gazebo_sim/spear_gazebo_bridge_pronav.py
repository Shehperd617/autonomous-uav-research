#!/usr/bin/env python3
"""
SPEAR Gazebo Bridge v3 — ProNav Guidance
=========================================
Uses your actual GuidanceComputer from guidance.py with ground-truth
pose data from Gazebo. Proves the guidance law works in 3D simulation.

Terminal 1:  ign gazebo two_drone_world.sdf  (press Play)
Terminal 2:  python3 spear_gazebo_bridge_pronav.py

Compares: Pure pursuit vs ProNav (run with --pursuit for baseline)
"""

import numpy as np
import subprocess
import signal
import time
import math
import sys
import re
import os

# Add parent dir so we can import SPEAR modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from guidance import GuidanceComputer


# ─── Config ──────────────────────────────────────────────────────────────────

WAYPOINTS = [
    (50, 20, 20),
    (30, 10, 18),
    (10, 5, 16),
    (0, 20, 18),
    (-10, -5, 20),
    (-20, 15, 15),
    (-15, 5, 10),
    (-5, 10, 22),
    (10, 15, 17),
]

TARGET_SPEED = 5.0
TARGET_SPEED_FAST = 8.0
WP_THRESHOLD = 4.0
INTERCEPT_DIST = 2.5
CONTROL_HZ = 10

USE_PRONAV = True  # False = pure pursuit baseline


# ─── Fake Tracker (feeds ground-truth to GuidanceComputer) ───────────────────

class SimTracker:
    """
    Simulates what the real Tracker would produce, using ground-truth
    positions from Gazebo instead of camera + YOLO + EKF.

    Provides the same interface that GuidanceComputer.compute() expects:
      .ok, .R, .Vc, .az, .el, .az_rate, .el_rate, .innovation_norm
    """

    def __init__(self):
        self.ok = False
        self.R = 0.0
        self.Vc = 0.0
        self.az = 0.0
        self.el = 0.0
        self.az_rate = 0.0
        self.el_rate = 0.0
        self.innovation_norm = 0.005  # Low = steady track

        self._prev_az = None
        self._prev_el = None
        self._prev_R = None
        self._prev_time = None

    def update_from_poses(self, int_pos, tgt_pos, t_now):
        """
        Compute tracker state from ground-truth positions.

        Args:
            int_pos: np.array([x, y, z]) interceptor position
            tgt_pos: np.array([x, y, z]) target position
        """
        # Relative position: target in interceptor's frame
        rel = tgt_pos - int_pos
        rx, ry, rz = rel[0], rel[1], rel[2]

        # Range
        self.R = np.linalg.norm(rel)
        if self.R < 0.01:
            return

        # Angles (az = horizontal bearing, el = vertical bearing)
        # In SPEAR convention: x=forward, y=right, z=down
        # Gazebo: x=forward, y=left, z=up — we adapt
        self.az = math.atan2(ry, rx)
        self.el = math.atan2(rz, rx)

        # Rates from finite difference
        dt = 0.0
        if self._prev_time is not None:
            dt = t_now - self._prev_time

        if dt > 0.01 and self._prev_az is not None:
            # LOS rates
            daz = self.az - self._prev_az
            # Wrap
            while daz > math.pi: daz -= 2 * math.pi
            while daz < -math.pi: daz += 2 * math.pi
            self.az_rate = daz / dt

            del_ = self.el - self._prev_el
            while del_ > math.pi: del_ -= 2 * math.pi
            while del_ < -math.pi: del_ += 2 * math.pi
            self.el_rate = del_ / dt

            # Closing velocity
            if self._prev_R is not None:
                self.Vc = -(self.R - self._prev_R) / dt  # positive = closing
            else:
                self.Vc = 0.0

            # Innovation norm — use change in LOS rates as proxy
            self.innovation_norm = abs(daz / dt) * 0.1
        else:
            self.az_rate = 0.0
            self.el_rate = 0.0
            self.Vc = 0.0

        self._prev_az = self.az
        self._prev_el = self.el
        self._prev_R = self.R
        self._prev_time = t_now
        self.ok = True


# ─── Gazebo Interface ────────────────────────────────────────────────────────

def publish_vel(topic, lx, ly, lz):
    """Fire-and-forget velocity publish."""
    msg = f'linear: {{x: {lx:.3f}, y: {ly:.3f}, z: {lz:.3f}}}'
    try:
        subprocess.Popen(
            ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Twist', '-p', msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def get_poses():
    """Get interceptor and target positions."""
    try:
        result = subprocess.run(
            ['ign', 'topic', '-t', '/world/spear_intercept/dynamic_pose/info',
             '-e', '-n', '1'],
            timeout=3.0, capture_output=True, text=True
        )
        int_pos = _parse_model_pos(result.stdout, "interceptor")
        tgt_pos = _parse_model_pos(result.stdout, "target")
        return int_pos, tgt_pos
    except Exception:
        return None, None


def _parse_model_pos(text, model_name):
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
    match = re.search(rf'{key}:\s*([-\d.e+]+)', text)
    return float(match.group(1)) if match else 0.0


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    mode = "PRONAV" if USE_PRONAV else "PURSUIT"

    print("=" * 58)
    print(f"  SPEAR Gazebo Bridge v3 — {mode} Guidance")
    print("=" * 58)
    print()

    # Init guidance
    gc = GuidanceComputer()
    tracker = SimTracker()

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
        print("\n  Failed. Is Gazebo running with Play pressed?")
        return

    # State
    wp_idx = 0
    intercepts = 0
    t_start = time.time()
    dt = 1.0 / CONTROL_HZ

    running = [True]
    def handler(s, f):
        running[0] = False
    signal.signal(signal.SIGINT, handler)

    print(f"\n  Mode: {mode}")
    print(f"  Target speed: {TARGET_SPEED}-{TARGET_SPEED_FAST} m/s")
    print(f"  Waypoints: {len(WAYPOINTS)}")
    print(f"  Intercept distance: {INTERCEPT_DIST}m")
    print(f"\n  Running... Ctrl+C to stop.\n")

    loop = 0
    intercept_cooldown = 0

    while running[0]:
        t0 = time.time()
        elapsed = t0 - t_start

        # Get poses
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
                print(f"  [TARGET] WP {wp_idx}/{len(WAYPOINTS)} Phase {phase}")
                if wp_idx >= len(WAYPOINTS):
                    wp_idx = 0
                continue

            speed = TARGET_SPEED if wp_idx < 3 else TARGET_SPEED_FAST
            tgt_vel = (diff / dist_to_wp) * speed
        else:
            wp_idx = 0
            tgt_vel = np.zeros(3)

        publish_vel("/model/target/cmd_vel", tgt_vel[0], tgt_vel[1], tgt_vel[2])

        # ── Update tracker with ground truth ──
        tracker.update_from_poses(int_pos, tgt_pos, elapsed)

        # ── Interceptor guidance ──
        separation = np.linalg.norm(tgt_pos - int_pos)

        if USE_PRONAV and tracker.ok:
            # ProNav guidance — outputs body-frame velocities
            vx_body, vy_body, vz_body = gc.compute(tracker, dt)

            # Convert body-frame to world-frame
            # Body x = toward target, y = lateral, z = vertical
            # We need to rotate body commands into world frame
            # using the current LOS direction as the body x-axis
            rel = tgt_pos - int_pos
            rel_norm = rel / max(np.linalg.norm(rel), 0.01)

            # Body x-axis = LOS direction
            bx = rel_norm

            # Body y-axis = perpendicular in horizontal plane
            up = np.array([0, 0, 1])
            by = np.cross(up, bx)
            by_norm = np.linalg.norm(by)
            if by_norm > 0.01:
                by = by / by_norm
            else:
                by = np.array([0, 1, 0])

            # Body z-axis = completes right-hand frame
            bz = np.cross(bx, by)

            # World velocity = body components projected onto world axes
            world_vel = vx_body * bx + vy_body * by + vz_body * bz

            # Scale up — guidance vx is tuned for video, sim needs more
            speed_scale = 1.5
            world_vel *= speed_scale

            # Ensure minimum closing speed
            closing_component = np.dot(world_vel, bx)
            if closing_component < 3.0 and separation > INTERCEPT_DIST * 2:
                world_vel += bx * (5.0 - closing_component)

            int_vel = world_vel

        else:
            # Pure pursuit fallback
            chase = tgt_pos - int_pos
            chase_dist = np.linalg.norm(chase)
            if chase_dist > 0.5:
                int_vel = (chase / chase_dist) * 12.0
            else:
                int_vel = np.zeros(3)

        publish_vel("/model/interceptor/cmd_vel",
                    int_vel[0], int_vel[1], int_vel[2])

        # ── Check intercept ──
        if intercept_cooldown > 0:
            intercept_cooldown -= 1

        if separation < INTERCEPT_DIST and intercept_cooldown == 0:
            intercepts += 1
            intercept_cooldown = 10  # 1 second cooldown at 10Hz
            print(f"\n  *** INTERCEPT #{intercepts} at t={elapsed:.1f}s "
                  f"| sep={separation:.1f}m | N={gc.N:.1f} "
                  f"| Tgo={gc.tgo:.2f}s ***\n")
            gc.reset()

        # ── Status ──
        if loop % 5 == 0:
            vc_str = f"Vc={tracker.Vc:+.1f}" if tracker.ok else "Vc=---"
            n_str = f"N={gc.N:.1f}" if USE_PRONAV else "PURSUIT"
            print(f"  t={elapsed:5.1f}s | "
                  f"Sep:{separation:5.1f}m | {vc_str} | {n_str} | "
                  f"Tgt({tgt_pos[0]:5.0f},{tgt_pos[1]:5.0f},{tgt_pos[2]:5.0f}) "
                  f"Int({int_pos[0]:5.0f},{int_pos[1]:5.0f},{int_pos[2]:5.0f}) "
                  f"WP:{wp_idx}/{len(WAYPOINTS)}")

        loop += 1

        sleep = dt - (time.time() - t0)
        if sleep > 0:
            time.sleep(sleep)

    # Summary
    runtime = time.time() - t_start
    print(f"\n{'=' * 58}")
    print(f"  RESULTS — {mode}")
    print(f"  Intercepts: {intercepts}")
    print(f"  Runtime: {runtime:.1f}s")
    if runtime > 0:
        print(f"  Rate: {intercepts / runtime * 60:.1f} intercepts/min")
    print(f"{'=' * 58}")

    publish_vel("/model/target/cmd_vel", 0, 0, 0)
    publish_vel("/model/interceptor/cmd_vel", 0, 0, 0)


if __name__ == "__main__":
    if "--pursuit" in sys.argv:
        USE_PRONAV = False
    main()
