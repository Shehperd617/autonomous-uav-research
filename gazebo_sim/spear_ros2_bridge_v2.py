#!/usr/bin/env python3
"""
SPEAR Bridge v2 — closed-loop interceptor with EKF + optional vision input.

Two operating modes:
  ORACLE  (default): target position comes from Gazebo TF (cheating)
  VISION  (--vision): target position comes from /spear/target_estimate
                      published by spear_yolo_eyes_v2.py

Both modes feed the target measurement through a 6-state TargetEKF
before passing it to ProNav3D guidance. This is the same EKF used in
the headless Monte Carlo harness, ported into the live system.

In VISION mode, missing detections are handled naturally by the
Kalman filter: it predicts forward from its current state until a
new measurement arrives.

Usage:
  python3 spear_ros2_bridge_v2.py            # oracle baseline
  python3 spear_ros2_bridge_v2.py --vision   # closed-loop vision
"""
import sys
import time
import argparse
import numpy as np
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from tf2_msgs.msg import TFMessage
except ImportError:
    print("Run: source /opt/ros/humble/setup.bash")
    sys.exit(1)

# ============ CONFIG ============
WAYPOINTS = [
    (50, 20, 20), (30, 10, 18), (10, 5, 16),
    (0, 20, 18), (-10, -5, 20), (-20, 15, 15),
    (-15, 5, 10), (-5, 10, 22), (10, 15, 17),
]
TGT_SPD = 8.0
TGT_SPD_FAST = 11.0
WP_THR = 5.0
HIT_DIST = 3.0
N_PN = 4.5
INT_SPD = 22.0
TERMINAL_R = 12.0
CONTROL_HZ = 20
FREEZE_TICKS = 15
REARM_DIST = 12.0
LEAD_GAIN = 0.8

# EKF parameters (matched to monte carlo v5.3 hard mode)
EKF_Q_POS = 0.5
EKF_Q_VEL = 25.0
EKF_R_BASE = 0.05
EKF_VISION_NOISE_STD = 1.5   # assume 1.5m std dev on vision measurements
EKF_ORACLE_NOISE_STD = 0.05  # near-zero for oracle


# ============ EKF (ported from spear_monte_carlo.py) ============
class TargetEKF:
    """6-state constant-velocity Kalman filter on [x,y,z,vx,vy,vz]."""
    def __init__(self, meas_noise_std=0.5):
        self.x = np.zeros(6)
        self.P = np.eye(6) * 100.0
        self.q_pos = EKF_Q_POS
        self.q_vel = EKF_Q_VEL
        self.R = np.eye(3) * (meas_noise_std ** 2 + EKF_R_BASE)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1
        self.initialized = False
        self.last_t = None
        self.last_meas_t = None

    def reset(self):
        self.initialized = False
        self.last_t = None
        self.last_meas_t = None
        self.x = np.zeros(6)
        self.P = np.eye(6) * 100.0

    def predict(self, dt):
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        self.x = F @ self.x
        Q = np.zeros((6, 6))
        Q[0, 0] = Q[1, 1] = Q[2, 2] = self.q_pos * dt
        Q[3, 3] = Q[4, 4] = Q[5, 5] = self.q_vel * dt
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def step_with_measurement(self, observed_pos, t):
        """Use this when a new measurement arrives."""
        if not self.initialized:
            self.x[0:3] = observed_pos
            self.x[3:6] = 0
            self.last_t = t
            self.last_meas_t = t
            self.initialized = True
            return self.x[0:3].copy(), self.x[3:6].copy()
        dt = t - self.last_t
        if dt > 0.001:
            self.predict(dt)
            self.update(observed_pos)
            self.last_t = t
            self.last_meas_t = t
        return self.x[0:3].copy(), self.x[3:6].copy()

    def step_predict_only(self, t):
        """Use this when no measurement arrived this tick — pure dead reckoning."""
        if not self.initialized:
            return None, None
        dt = t - self.last_t
        if dt > 0.001:
            self.predict(dt)
            self.last_t = t
        return self.x[0:3].copy(), self.x[3:6].copy()

    def time_since_measurement(self, t):
        if self.last_meas_t is None:
            return float('inf')
        return t - self.last_meas_t


# ============ GUIDANCE (unchanged from v17) ============
class ProNav3D:
    def __init__(self):
        self.prev_los = None
        self.prev_R = None
        self.prev_t = None
        self.prev_tp = None
        self.Vc = 0
        self.R = 0
        self.tgo = 999
        self.zem = 0
        self.tgt_vel = np.zeros(3)

    def reset(self):
        self.prev_los = None
        self.prev_R = None
        self.prev_t = None
        self.prev_tp = None
        self.Vc = 0
        self.zem = 0
        self.tgt_vel = np.zeros(3)

    def compute(self, ip, tp, t, ekf_tgt_vel=None):
        los = tp - ip
        self.R = np.linalg.norm(los)
        if self.R < 0.1:
            return np.zeros(3)
        lh = los / self.R

        # Use EKF velocity estimate if provided, otherwise estimate from positions
        if ekf_tgt_vel is not None:
            self.tgt_vel = ekf_tgt_vel
        elif self.prev_tp is not None and self.prev_t is not None:
            dt_v = t - self.prev_t
            if 0.005 < dt_v < 0.5:
                new_tv = (tp - self.prev_tp) / dt_v
                self.tgt_vel = 0.7 * self.tgt_vel + 0.3 * new_tv

        if self.prev_los is not None and self.prev_t is not None:
            dt = t - self.prev_t
            if 0.005 < dt < 0.5:
                dlos = (lh - self.prev_los) / dt
                self.Vc = -(self.R - self.prev_R) / dt if self.prev_R else 0
                vc = max(self.Vc, 4.0)
                self.tgo = self.R / vc
                lrp = dlos - np.dot(dlos, lh) * lh
                self.zem = self.R * np.linalg.norm(lrp) * self.tgo
                if self.R < TERMINAL_R:
                    aim = tp + self.tgt_vel * self.tgo * LEAD_GAIN
                    lead_los = aim - ip
                    ll = np.linalg.norm(lead_los)
                    if ll > 0.1:
                        self._s(lh, t, tp)
                        return (lead_los / ll) * INT_SPD
                    self._s(lh, t, tp)
                    return lh * INT_SPD
                a = N_PN * vc * lrp
                corr = a * min(self.tgo, 1.5)
                cm = np.linalg.norm(corr)
                if cm > INT_SPD * 0.7:
                    corr = corr / cm * INT_SPD * 0.7
                aim = tp + self.tgt_vel * self.tgo * LEAD_GAIN
                lead_dir = aim - ip
                ld = np.linalg.norm(lead_dir)
                base = lead_dir / ld if ld > 0.1 else lh
                v = base * INT_SPD + corr
                s = np.linalg.norm(v)
                if s > INT_SPD * 1.2:
                    v = v / s * INT_SPD * 1.2
                self._s(lh, t, tp)
                return v
        self._s(lh, t, tp)
        return lh * INT_SPD

    def _s(self, lh, t, tp):
        self.prev_los = lh.copy()
        self.prev_R = self.R
        self.prev_t = t
        self.prev_tp = tp.copy()


# ============ NODE ============
class SpearNodeV2(Node):
    def __init__(self, use_vision=False):
        super().__init__('spear_bridge_v2')
        self.use_vision = use_vision

        # Publishers
        self.int_pub = self.create_publisher(
            Twist, '/model/interceptor/cmd_vel', 10)
        self.tgt_pub = self.create_publisher(
            Twist, '/model/target/cmd_vel', 10)

        # Pose state from oracle TF
        self.int_pos = None
        self.tgt_pos_oracle = None  # always available, used for hit detection
                                    # and for the target's own waypoint chase

        # Vision target state (only used if --vision)
        self.tgt_pos_vision = None
        self.vision_count = 0
        self.last_vision_t = None

        # EKF for guidance
        meas_noise = (EKF_VISION_NOISE_STD if use_vision
                      else EKF_ORACLE_NOISE_STD)
        self.ekf = TargetEKF(meas_noise_std=meas_noise)

        # Subscribers
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/spear_intercept/dynamic_pose/info',
            self.pose_cb, 10)
        if use_vision:
            self.vision_sub = self.create_subscription(
                PoseStamped,
                '/spear/target_estimate',
                self.vision_cb, 10)

        # Timers
        self.cmd_timer = self.create_timer(1.0 / CONTROL_HZ, self.tick)
        self.pub_timer = self.create_timer(0.05, self.republish)

        # State
        self.nav = ProNav3D()
        self.wp = 0
        self.hits = 0
        self.t0 = time.time()
        self.cd = 0
        self.lp = 0
        self.pose_count = 0
        self.last_iv = np.zeros(3)
        self.last_tv = np.zeros(3)
        self.armed = True

        mode = "VISION (closed-loop YOLO)" if use_vision else "ORACLE (cheating TF)"
        self.get_logger().info(f"SPEAR v2 — {CONTROL_HZ}Hz — mode: {mode}")
        self.get_logger().info(f"EKF measurement noise std: {meas_noise:.3f}m")

    def pose_cb(self, msg):
        for tf in msg.transforms:
            n = tf.child_frame_id
            p = tf.transform.translation
            if n == 'interceptor':
                self.int_pos = np.array([p.x, p.y, p.z])
            elif n == 'target':
                self.tgt_pos_oracle = np.array([p.x, p.y, p.z])
        self.pose_count += 1

    def vision_cb(self, msg):
        p = msg.pose.position
        self.tgt_pos_vision = np.array([p.x, p.y, p.z])
        self.vision_count += 1
        self.last_vision_t = time.time() - self.t0

    def pub_vel(self, publisher, v):
        m = Twist()
        m.linear.x = float(v[0])
        m.linear.y = float(v[1])
        m.linear.z = float(v[2])
        publisher.publish(m)

    def republish(self):
        self.pub_vel(self.int_pub, self.last_iv)
        self.pub_vel(self.tgt_pub, self.last_tv)

    def get_target_measurement(self, t):
        """
        Returns (measurement_pos, has_new_measurement) for the EKF.
        - In oracle mode: always returns oracle pose as a fresh measurement.
        - In vision mode: returns vision pose only if a new vision message
          arrived since the last tick. Otherwise returns None to indicate
          'predict only'.
        """
        if not self.use_vision:
            return self.tgt_pos_oracle, True

        # Vision mode: only feed measurement if vision msg arrived recently
        if self.tgt_pos_vision is None:
            return None, False
        if self.last_vision_t is None:
            return None, False
        if t - self.last_vision_t > 0.5:
            # Stale vision data — let EKF predict only
            return None, False
        return self.tgt_pos_vision, True

    def tick(self):
        el = time.time() - self.t0

        if self.int_pos is None or self.tgt_pos_oracle is None:
            if self.lp % 20 == 0:
                self.get_logger().warn(
                    f"Waiting for poses... ({self.pose_count})")
            self.lp += 1
            return

        ip = self.int_pos.copy()
        tp_oracle = self.tgt_pos_oracle.copy()  # for hit detection always

        # Target waypoint chase (always uses oracle — the target is autonomous)
        w = np.array(WAYPOINTS[self.wp])
        d = w - tp_oracle
        dn = np.linalg.norm(d)
        if dn < WP_THR:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            ph = 1 if self.wp < 3 else (2 if self.wp < 6 else 3)
            self.get_logger().info(
                f"[TGT] WP{self.wp}/{len(WAYPOINTS)} Ph{ph}")
            w = np.array(WAYPOINTS[self.wp])
            d = w - tp_oracle
            dn = np.linalg.norm(d)
        s = TGT_SPD if self.wp < 3 else TGT_SPD_FAST
        self.last_tv = (d / dn) * s if dn > 0.1 else np.zeros(3)

        # === EKF: feed measurement (or predict only) ===
        meas, has_new = self.get_target_measurement(el)
        if has_new:
            tp_filtered, tv_filtered = self.ekf.step_with_measurement(meas, el)
        else:
            tp_filtered, tv_filtered = self.ekf.step_predict_only(el)

        # If EKF isn't initialized yet (vision mode, no detections yet),
        # fall back to standing still
        if tp_filtered is None:
            self.last_iv = np.zeros(3)
            if self.lp % 20 == 0:
                self.get_logger().warn(
                    f"EKF uninitialized — waiting for first measurement "
                    f"(vision_count={self.vision_count})")
            self.lp += 1
            return

        # Hit detection ALWAYS uses true oracle position — we want to know
        # if a real interception happened, regardless of what the perception
        # system thinks
        sep_true = np.linalg.norm(tp_oracle - ip)

        # Interceptor freeze/rearm logic
        if self.cd > 0:
            self.cd -= 1
            self.last_iv = np.zeros(3)
            if self.cd == 0:
                self.nav.reset()
                self.armed = True
        else:
            if not self.armed and sep_true > REARM_DIST:
                self.armed = True
            if self.armed:
                # Guidance uses EKF-filtered target position and velocity
                iv = self.nav.compute(ip, tp_filtered, el,
                                       ekf_tgt_vel=tv_filtered)
                if sep_true > 80.0:
                    chase = tp_filtered - ip
                    cn = np.linalg.norm(chase)
                    if cn > 1.0:
                        iv = (chase / cn) * INT_SPD
                sp = np.linalg.norm(iv)
                if sp > INT_SPD * 1.5:
                    iv = iv / sp * INT_SPD * 1.5
                self.last_iv = iv
            else:
                self.last_iv = np.zeros(3)

        # Hit detection (against TRUE position, not estimated)
        if sep_true < HIT_DIST and self.cd == 0 and self.armed:
            self.hits += 1
            self.cd = FREEZE_TICKS
            self.armed = False
            self.last_iv = np.zeros(3)
            self.get_logger().info(
                f"*** HIT#{self.hits} t={el:.1f}s sep={sep_true:.1f}m ***")

        # Status output
        if self.lp % 10 == 0:
            ph = "PN" if sep_true >= TERMINAL_R else "TM"
            if self.cd > 0:
                ph = "HIT"
            elif not self.armed:
                ph = "RST"
            # Show estimation error: how far the EKF thinks the target is from
            # where it actually is
            est_err = np.linalg.norm(tp_filtered - tp_oracle)
            mode_tag = "V" if self.use_vision else "O"
            extra = f"vis={self.vision_count}" if self.use_vision else ""
            self.get_logger().info(
                f"t={el:5.1f}s [{mode_tag}] Sep:{sep_true:5.1f}m "
                f"EstErr:{est_err:.2f}m Vc={self.nav.Vc:+.1f} "
                f"ZEM={self.nav.zem:.1f} [{ph}] WP:{self.wp} "
                f"H:{self.hits} {extra}"
            )
        self.lp += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision', action='store_true',
                        help='use /spear/target_estimate as EKF input '
                             'instead of oracle TF (closed-loop vision mode)')
    args = parser.parse_args()

    print("=" * 60)
    mode = "VISION (closed-loop)" if args.vision else "ORACLE (baseline)"
    print(f"  SPEAR v2 — EKF + ProNav3D — mode: {mode}")
    print("=" * 60)

    rclpy.init()
    node = SpearNodeV2(use_vision=args.vision)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        el = time.time() - node.t0
        rate = node.hits / max(el, 1) * 60
        print(f"\n  {node.hits} hits in {el:.0f}s ({rate:.1f}/min)")
        if args.vision:
            print(f"  Vision detections received: {node.vision_count}")
    finally:
        node.pub_vel(node.int_pub, np.zeros(3))
        node.pub_vel(node.tgt_pub, np.zeros(3))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
