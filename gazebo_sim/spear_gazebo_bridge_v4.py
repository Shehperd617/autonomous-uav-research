#!/usr/bin/env python3
"""
SPEAR Gazebo Bridge v4 — Direct ProNav in World Frame
======================================================
Implements Proportional Navigation directly in 3D world coordinates.
No body-frame conversion — cleaner and matches missile guidance textbooks.

  a_cmd = N * Vc * (Omega x LOS)

where Omega is the LOS rotation rate vector and LOS is the unit line-of-sight.

Terminal 1:  ign gazebo two_drone_world.sdf  (press Play)
Terminal 2:  python3 spear_gazebo_bridge_v4.py
Terminal 2:  python3 spear_gazebo_bridge_v4.py --pursuit   (baseline comparison)
"""

import numpy as np
import subprocess
import signal
import time
import math
import sys
import re


# ─── Config ──────────────────────────────────────────────────────────────────

WAYPOINTS = [
    (50, 20, 20), (30, 10, 18), (10, 5, 16),       # Phase 1: straight
    (0, 20, 18), (-10, -5, 20), (-20, 15, 15),      # Phase 2: evasive
    (-15, 5, 10), (-5, 10, 22), (10, 15, 17),       # Phase 3: aggressive
]

TARGET_SPEED      = 5.0
TARGET_SPEED_FAST = 8.0
WP_THRESHOLD      = 4.0
INTERCEPT_DIST    = 2.5
CONTROL_HZ        = 10

# ProNav parameters
PN_N              = 4.0       # Navigation constant (3-5 typical)
INTERCEPTOR_SPEED = 12.0      # Max interceptor speed m/s
MIN_CLOSING_SPEED = 3.0       # Minimum closing speed to maintain

USE_PRONAV = "--pursuit" not in sys.argv


# ─── 3D ProNav Computer ─────────────────────────────────────────────────────

class ProNav3D:
    """
    True 3D Proportional Navigation in world frame.

    Standard ProNav: acceleration perpendicular to LOS, proportional to
    LOS rate and closing velocity.

    a_cmd = N * Vc * (Omega x LOS_hat)

    This is the actual guidance law from missile textbooks, not an
    approximation through body-frame angles.
    """

    def __init__(self, N=4.0):
        self.N = N
        self._prev_los = None
        self._prev_time = None

        # Telemetry
        self.los_rate_mag = 0.0
        self.Vc = 0.0
        self.R = 0.0
        self.tgo = 999.0
        self.zem = 0.0

    def compute(self, int_pos, tgt_pos, t_now):
        """
        Compute ProNav velocity command in world frame.

        Returns: velocity vector (np.array[3]) for the interceptor
        """
        # LOS vector and range
        los_vec = tgt_pos - int_pos
        self.R = np.linalg.norm(los_vec)

        if self.R < 0.1:
            return np.zeros(3)

        los_hat = los_vec / self.R  # Unit LOS vector

        # LOS rate (angular velocity of LOS)
        if self._prev_los is not None and self._prev_time is not None:
            dt = t_now - self._prev_time
            if dt > 0.01:
                # LOS rate = d(los_hat)/dt
                dlos = los_hat - self._prev_los
                los_rate = dlos / dt

                # Closing velocity = -dR/dt
                prev_R = np.linalg.norm(
                    (tgt_pos - int_pos) + (los_hat - self._prev_los) * self.R)
                # Actually simpler: Vc from range change
                self.Vc = -(self.R - np.linalg.norm(
                    los_vec + self._prev_los * self._prev_R)) / dt if hasattr(self, '_prev_R') else 0.0

                # Better Vc calculation: project relative velocity onto LOS
                # We don't have velocities directly, so use finite difference of R
                if hasattr(self, '_prev_R'):
                    self.Vc = -(self.R - self._prev_R) / dt

                # Time to go
                Vc_safe = max(self.Vc, 1.0)
                self.tgo = self.R / Vc_safe

                # LOS rotation rate magnitude
                self.los_rate_mag = np.linalg.norm(los_rate)

                # Zero Effort Miss
                self.zem = self.R * self.los_rate_mag * self.tgo

                # ── ProNav acceleration ──
                # a = N * Vc * los_rate (component perpendicular to LOS)
                # Remove component along LOS from los_rate
                los_rate_perp = los_rate - np.dot(los_rate, los_hat) * los_hat

                # ProNav lateral acceleration
                a_cmd = self.N * max(self.Vc, MIN_CLOSING_SPEED) * los_rate_perp

                # Build velocity command:
                # 1. Fly toward target at INTERCEPTOR_SPEED along LOS
                # 2. Add ProNav correction perpendicular to LOS
                v_closing = los_hat * INTERCEPTOR_SPEED

                # Scale ProNav correction (it's acceleration, integrate to velocity)
                # In discrete time: v_correction = a_cmd * dt
                # But we want steady-state correction, so scale by tgo
                correction_gain = min(self.tgo, 2.0)  # Cap gain
                v_correction = a_cmd * correction_gain

                # Combine
                v_cmd = v_closing + v_correction

                # Limit total speed
                speed = np.linalg.norm(v_cmd)
                if speed > INTERCEPTOR_SPEED * 1.5:
                    v_cmd = v_cmd / speed * INTERCEPTOR_SPEED * 1.5

                self._prev_los = los_hat.copy()
                self._prev_R = self.R
                self._prev_time = t_now

                return v_cmd

        # First frame — just fly toward target
        self._prev_los = los_hat.copy()
        self._prev_R = self.R
        self._prev_time = t_now
        return los_hat * INTERCEPTOR_SPEED

    def reset(self):
        self._prev_los = None
        self._prev_time = None
        self._prev_R = None
        self.los_rate_mag = 0.0
        self.Vc = 0.0


# ─── Gazebo Interface ────────────────────────────────────────────────────────

def publish_vel(topic, lx, ly, lz):
    msg = f'linear: {{x: {lx:.3f}, y: {ly:.3f}, z: {lz:.3f}}}'
    try:
        subprocess.Popen(
            ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Twist', '-p', msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def get_poses():
    try:
        result = subprocess.run(
            ['ign', 'topic', '-t', '/world/spear_intercept/dynamic_pose/info',
             '-e', '-n', '1'],
            timeout=3.0, capture_output=True, text=True
        )
        return _parse_model_pos(result.stdout, "interceptor"), \
               _parse_model_pos(result.stdout, "target")
    except:
        return None, None


def _parse_model_pos(text, model_name):
    match = re.search(
        rf'name: "{model_name}".*?position\s*\{{(.*?)\}}', text, re.DOTALL)
    if not match:
        return None
    b = match.group(1)
    return np.array([_v(b,'x'), _v(b,'y'), _v(b,'z')])


def _v(text, key):
    m = re.search(rf'{key}:\s*([-\d.e+]+)', text)
    return float(m.group(1)) if m else 0.0


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    mode = "PRONAV (N=%.1f)" % PN_N if USE_PRONAV else "PURE PURSUIT"

    print("=" * 58)
    print(f"  SPEAR Sim v4 — {mode}")
    print("=" * 58)

    nav = ProNav3D(N=PN_N)

    # Connect
    print("  Connecting...", end='', flush=True)
    for _ in range(20):
        ip, tp = get_poses()
        if ip is not None:
            print(" OK!")
            break
        print(".", end='', flush=True)
        time.sleep(1)
    else:
        print(" FAILED"); return

    print(f"  Interceptor: {ip}")
    print(f"  Target:      {tp}")
    print(f"\n  Running... Ctrl+C to stop.\n")

    wp_idx = 0
    intercepts = 0
    t_start = time.time()
    dt = 1.0 / CONTROL_HZ
    cooldown = 0

    running = [True]
    signal.signal(signal.SIGINT, lambda s,f: running.__setitem__(0, False))

    loop = 0
    while running[0]:
        t0 = time.time()
        elapsed = t0 - t_start

        ip, tp = get_poses()
        if ip is None or tp is None:
            time.sleep(dt); continue

        # ── Target waypoints ──
        if wp_idx < len(WAYPOINTS):
            wp = np.array(WAYPOINTS[wp_idx])
            diff = wp - tp
            d = np.linalg.norm(diff)
            if d < WP_THRESHOLD:
                wp_idx = (wp_idx + 1) % len(WAYPOINTS)
                phase = 1 if wp_idx < 3 else (2 if wp_idx < 6 else 3)
                print(f"  [TGT] WP {wp_idx}/{len(WAYPOINTS)} Phase {phase}")
                continue
            spd = TARGET_SPEED if wp_idx < 3 else TARGET_SPEED_FAST
            tv = (diff / d) * spd
        else:
            wp_idx = 0; tv = np.zeros(3)

        publish_vel("/model/target/cmd_vel", tv[0], tv[1], tv[2])

        # ── Interceptor ──
        sep = np.linalg.norm(tp - ip)

        if USE_PRONAV:
            iv = nav.compute(ip, tp, elapsed)
        else:
            # Pure pursuit
            d = tp - ip
            dn = np.linalg.norm(d)
            iv = (d / dn * INTERCEPTOR_SPEED) if dn > 0.5 else np.zeros(3)

        publish_vel("/model/interceptor/cmd_vel", iv[0], iv[1], iv[2])

        # ── Intercept check ──
        if cooldown > 0:
            cooldown -= 1
        if sep < INTERCEPT_DIST and cooldown == 0:
            intercepts += 1
            cooldown = 15
            nav.reset()
            print(f"\n  *** INTERCEPT #{intercepts} t={elapsed:.1f}s "
                  f"sep={sep:.1f}m Vc={nav.Vc:.1f} ***\n")

        # ── Status ──
        if loop % 5 == 0:
            if USE_PRONAV:
                print(f"  t={elapsed:5.1f}s | Sep:{sep:5.1f}m | "
                      f"Vc={nav.Vc:+5.1f} | LOS_rate={nav.los_rate_mag:.3f} | "
                      f"Tgo={nav.tgo:5.1f} | ZEM={nav.zem:5.1f} | "
                      f"WP:{wp_idx}/{len(WAYPOINTS)}")
            else:
                print(f"  t={elapsed:5.1f}s | Sep:{sep:5.1f}m | "
                      f"WP:{wp_idx}/{len(WAYPOINTS)}")

        loop += 1
        sl = dt - (time.time() - t0)
        if sl > 0: time.sleep(sl)

    rt = time.time() - t_start
    print(f"\n{'='*58}")
    print(f"  {mode}: {intercepts} intercepts in {rt:.0f}s "
          f"({intercepts/rt*60:.1f}/min)")
    print(f"{'='*58}")
    publish_vel("/model/target/cmd_vel", 0, 0, 0)
    publish_vel("/model/interceptor/cmd_vel", 0, 0, 0)


if __name__ == "__main__":
    main()
