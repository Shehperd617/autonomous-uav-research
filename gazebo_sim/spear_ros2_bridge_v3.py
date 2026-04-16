#!/usr/bin/env python3
"""
SPEAR Bridge v3.0 — CUED LAUNCH + SENSOR FUSION.

CHANGES FROM v2.5:
  - Acoustic bearing now read as WORLD frame (no body conversion).
    Eliminates oscillation and "flying backward" during HUNT.
  - Post-hit cooldown ends with immediate re-engagement: spear acts
    on whichever sensor is fresh (vision > acoustic > last-known)
    instead of sitting still until the next state transition.

3-state machine:
  TRACKING  : vision fresh, full ProNav
  COASTING  : vision stale 0.5-1.5s, half-speed ProNav on EKF predict
  SEARCHING : vision lost. Use acoustic world-bearing to fly toward
              target. Vision homes when reacquired.
"""
import sys
import math
import time
import argparse
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped, Vector3
    from tf2_msgs.msg import TFMessage
except ImportError:
    print("Run: source /opt/ros/humble/setup.bash")
    sys.exit(1)

# ============ CONFIG ============
WAYPOINTS = [
    (50, 25, 20), (55, 20, 21), (52, 28, 19),
    (48, 22, 21), (53, 26, 20), (50, 18, 20),
    (47, 24, 21), (54, 22, 19), (51, 27, 20),
]
TGT_SPD = 4.0
TGT_SPD_FAST = 5.0
WP_THR = 3.0

# ============ LAUNCH CONFIG ============
LAUNCH_NOISE_DEG = 7.0      # launcher bearing noise (degrees)
LAUNCH_SPEED = 22.0         # launch velocity (m/s)
LAUNCH_MAX_TIME = 3.0       # max time in LAUNCH state
HIT_DIST = 3.0
N_PN = 4.5
INT_SPD = 22.0
TERMINAL_R = 12.0
CONTROL_HZ = 20
FREEZE_TICKS = 15
REARM_DIST = 12.0
LEAD_GAIN = 0.8

EKF_Q_POS = 0.5
EKF_Q_VEL = 25.0
EKF_R_BASE = 0.05
EKF_VISION_NOISE_STD = 1.0
EKF_ORACLE_NOISE_STD = 0.05

STALE_THRESHOLD = 0.5
LOST_THRESHOLD = 1.5

SEARCH_CHASE_SPEED = 22.0
SEARCH_YAW_RATE = 0.6
SEARCH_MIN_DIST = 1.5

ACOUSTIC_STALE_THRESHOLD = 1.0
ACOUSTIC_MIN_SIGNAL = 0.05

ALT_MIN = 10.0
ALT_MAX = 30.0


# ============ EKF ============
class TargetEKF:
    def __init__(self, meas_noise_std=0.5):
        self.meas_noise_std = meas_noise_std
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


# ============ GUIDANCE ============
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
        super().__init__('spear_bridge_v3')
        self.use_vision = use_vision
        self.shutdown_requested = False

        self.int_pub = self.create_publisher(
            Twist, '/model/interceptor/cmd_vel', 10)
        self.tgt_pub = self.create_publisher(
            Twist, '/model/target/cmd_vel', 10)

        self.int_pos = None
        self.tgt_pos_oracle = None

        self.tgt_pos_vision = None
        self.vision_count = 0
        self.last_vision_t = None
        self.last_known_tgt_pos = None

        # Acoustic state — now WORLD FRAME
        self.acoustic_az_world = None
        self.acoustic_el_world = None
        self.acoustic_signal = 0.0
        self.acoustic_count = 0
        self.last_acoustic_t = None

        meas_noise = (EKF_VISION_NOISE_STD if use_vision
                      else EKF_ORACLE_NOISE_STD)
        self.ekf = TargetEKF(meas_noise_std=meas_noise)

        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/spear_intercept/dynamic_pose/info',
            self.pose_cb, 10)
        if use_vision:
            self.vision_sub = self.create_subscription(
                PoseStamped,
                '/spear/target_estimate',
                self.vision_cb, 10)
        self.acoustic_sub = self.create_subscription(
            Vector3,
            '/spear/acoustic_bearing',
            self.acoustic_cb, 10)

        self.cmd_timer = self.create_timer(1.0 / CONTROL_HZ, self.tick)
        self.pub_timer = self.create_timer(0.05, self.republish)

        self.nav = ProNav3D()
        self.wp = 0
        self.hits = 0
        self.t0 = time.time()
        self.cd = 0
        self.lp = 0
        self.pose_count = 0
        self.last_iv = np.zeros(3)
        self.last_iv_angular = np.zeros(3)
        self.last_tv = np.zeros(3)
        self.armed = True

        self.state = "LAUNCH"
        self.launch_bearing = None
        self.launch_t = None

        mode = "VISION (closed-loop YOLO)" if use_vision else "ORACLE (cheating TF)"
        self.get_logger().info(f"SPEAR v3.0 — {CONTROL_HZ}Hz — mode: {mode}")
        self.get_logger().info(f"EKF measurement noise std: {meas_noise:.3f}m")
        self.get_logger().info(
            f"State thresholds: COASTING>{STALE_THRESHOLD}s  "
            f"SEARCHING>{LOST_THRESHOLD}s")
        self.get_logger().info(
            "SEARCH: world-frame acoustic cueing (no oscillation)")
        self.get_logger().info(f"Altitude cap: {ALT_MIN}-{ALT_MAX}m")

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
        self.last_known_tgt_pos = self.tgt_pos_vision.copy()
        self.vision_count += 1
        self.last_vision_t = time.time() - self.t0

    def acoustic_cb(self, msg):
        if msg.z < ACOUSTIC_MIN_SIGNAL:
            return
        self.acoustic_az_world = msg.x
        self.acoustic_el_world = msg.y
        self.acoustic_signal = msg.z
        self.acoustic_count += 1
        self.last_acoustic_t = time.time() - self.t0

    def acoustic_fresh(self, t):
        if self.last_acoustic_t is None:
            return False
        return (t - self.last_acoustic_t) < ACOUSTIC_STALE_THRESHOLD

    def acoustic_world_velocity(self, speed):
        """Convert WORLD (az, el) bearing -> world-frame velocity."""
        az = self.acoustic_az_world
        el = self.acoustic_el_world
        if az is None or el is None:
            return None
        cx = math.cos(el) * math.cos(az)
        cy = math.cos(el) * math.sin(az)
        cz = math.sin(el)
        return np.array([cx, cy, cz]) * speed

    def apply_altitude_cap(self, ip, v):
        if ip[2] >= ALT_MAX and v[2] > 0:
            v = v.copy()
            v[2] = 0
        elif ip[2] <= ALT_MIN and v[2] < 0:
            v = v.copy()
            v[2] = 0
        return v

    def pub_vel(self, publisher, v_linear, v_angular=None):
        if self.shutdown_requested:
            return
        try:
            m = Twist()
            m.linear.x = float(v_linear[0])
            m.linear.y = float(v_linear[1])
            m.linear.z = float(v_linear[2])
            if v_angular is not None:
                m.angular.x = float(v_angular[0])
                m.angular.y = float(v_angular[1])
                m.angular.z = float(v_angular[2])
            publisher.publish(m)
        except Exception:
            pass

    def republish(self):
        self.pub_vel(self.int_pub, self.last_iv, self.last_iv_angular)
        self.pub_vel(self.tgt_pub, self.last_tv)

    def get_target_measurement(self, t):
        if not self.use_vision:
            return self.tgt_pos_oracle, True
        if self.tgt_pos_vision is None:
            return None, False
        if self.last_vision_t is None:
            return None, False
        if t - self.last_vision_t > STALE_THRESHOLD:
            return None, False
        return self.tgt_pos_vision, True

    def compute_state(self, t):
        if not self.use_vision:
            return "TRACKING"
        if self.last_vision_t is None:
            return "SEARCHING"
        age = t - self.last_vision_t
        if age < STALE_THRESHOLD:
            return "TRACKING"
        elif age < LOST_THRESHOLD:
            return "COASTING"
        else:
            return "SEARCHING"

    def compute_search_velocity(self, ip, el_t):
        """Compute velocity for SEARCHING state. Returns (iv, src_tag).

        Priority: acoustic > last-known > cold-start fallback.
        Used both during normal SEARCHING and post-HIT re-engagement.
        """
        # PRIORITY 1: acoustic
        if self.acoustic_fresh(el_t) and self.acoustic_az_world is not None:
            iv = self.acoustic_world_velocity(SEARCH_CHASE_SPEED)
            if iv is not None:
                return iv, "ACC"
        # PRIORITY 2: last-known vision position
        if self.last_known_tgt_pos is not None:
            chase_vec = self.last_known_tgt_pos - ip
            chase_dist = np.linalg.norm(chase_vec)
            if chase_dist > SEARCH_MIN_DIST:
                return (chase_vec / chase_dist) * SEARCH_CHASE_SPEED, "LK"
            return np.zeros(3), "LK"
        # PRIORITY 3: continue on launch bearing
        if self.launch_bearing is not None:
            return self.launch_bearing * SEARCH_CHASE_SPEED, "LB"
        return np.zeros(3), "NONE"

    def tick(self):
        el_t = time.time() - self.t0

        if self.int_pos is None or self.tgt_pos_oracle is None:
            if self.lp % 20 == 0:
                self.get_logger().warn(
                    f"Waiting for poses... ({self.pose_count})")
            self.lp += 1
            return

        ip = self.int_pos.copy()
        tp_oracle = self.tgt_pos_oracle.copy()
# Compute launch bearing on first tick
        if self.launch_bearing is None:
            true_dir = tp_oracle - ip
            true_dist = np.linalg.norm(true_dir)
            if true_dist > 0.1:
                true_unit = true_dir / true_dist
                noise_rad = math.radians(LAUNCH_NOISE_DEG)
                az = math.atan2(true_dir[1], true_dir[0]) + np.random.normal(0, noise_rad)
                el = math.atan2(true_dir[2], math.sqrt(true_dir[0]**2 + true_dir[1]**2)) + np.random.normal(0, noise_rad)
                self.launch_bearing = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
                err = math.degrees(math.acos(np.clip(np.dot(true_unit, self.launch_bearing), -1, 1)))
                self.get_logger().info(f"LAUNCH BEARING: err={err:.1f}° range={true_dist:.1f}m")
            else:
                self.launch_bearing = np.array([1.0, 0.0, 0.0])
            self.launch_t = el_t

        # Target waypoint chase
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

        # === EKF update ===
        meas, has_new = self.get_target_measurement(el_t)

        # State transition logic
        new_state = self.compute_state(el_t)
        if new_state != self.state:
            age = (el_t - self.last_vision_t) if self.last_vision_t else -1
            self.get_logger().warn(
                f"State: {self.state} -> {new_state}  "
                f"(age={age:.2f}s  vis={self.vision_count} "
                f"acc={self.acoustic_count})")

            if self.state == "SEARCHING" and new_state == "TRACKING":
                self.get_logger().warn("EKF RESET — fresh vision lock")
                self.ekf.reset()
                self.nav.reset()

            if new_state == "SEARCHING":
                self.search_entered_t = el_t

            self.state = new_state

        if has_new:
            tp_filtered, tv_filtered = self.ekf.step_with_measurement(meas, el_t)
        else:
            tp_filtered, tv_filtered = self.ekf.step_predict_only(el_t)

        sep_true = np.linalg.norm(tp_oracle - ip)

        # === Command computation per state ===
        if self.cd > 0:
            self.cd -= 1
            self.last_iv = np.zeros(3)
            self.last_iv_angular = np.zeros(3)
            if self.cd == 0:
                self.nav.reset()
                self.armed = True
                # POST-HIT RE-ENGAGE: don't sit still, immediately
                # compute a search velocity using whatever sensor is fresh.
                iv, src = self.compute_search_velocity(ip, el_t)
                iv = self.apply_altitude_cap(ip, iv)
                self.last_iv = iv
                self.last_iv_angular = np.array([0.0, 0.0, SEARCH_YAW_RATE])
                self._search_src = f"REENG-{src}"
        else:
            if not self.armed and sep_true > REARM_DIST:
                self.armed = True

            if not self.armed:
                self.last_iv = np.zeros(3)
                self.last_iv_angular = np.zeros(3)

            elif self.state == "SEARCHING":
                iv, src = self.compute_search_velocity(ip, el_t)
                iv = self.apply_altitude_cap(ip, iv)
                self.last_iv = iv
                self.last_iv_angular = np.array([0.0, 0.0, SEARCH_YAW_RATE])
                self._search_src = src
            else:
                # TRACKING or COASTING
                if tp_filtered is None:
                    self.last_iv = np.zeros(3)
                    self.last_iv_angular = np.zeros(3)
                else:
                    iv = self.nav.compute(ip, tp_filtered, el_t,
                                           ekf_tgt_vel=tv_filtered)
                    if self.state == "COASTING":
                        iv = iv * 0.5
                    if self.state == "TRACKING" and sep_true > 80.0:
                        chase = tp_filtered - ip
                        cn = np.linalg.norm(chase)
                        if cn > 1.0:
                            iv = (chase / cn) * INT_SPD
                    sp = np.linalg.norm(iv)
                    if sp > INT_SPD * 1.5:
                        iv = iv / sp * INT_SPD * 1.5
                    iv = self.apply_altitude_cap(ip, iv)
                    self.last_iv = iv
                    self.last_iv_angular = np.zeros(3)

        # Hit detection
        if sep_true < HIT_DIST and self.cd == 0 and self.armed:
            self.hits += 1
            self.cd = FREEZE_TICKS
            self.armed = False
            self.last_iv = np.zeros(3)
            self.last_iv_angular = np.zeros(3)
            self.get_logger().info(
                f"*** HIT#{self.hits} t={el_t:.1f}s sep={sep_true:.1f}m ***")

        # Status log
        if self.lp % 10 == 0:
            ph = "PN" if sep_true >= TERMINAL_R else "TM"
            if self.cd > 0:
                ph = "HIT"
            elif not self.armed:
                ph = "RST"
            elif self.state == "SEARCHING":
                ph = f"HUNT-{getattr(self, '_search_src', '?')}"
            elif self.state == "COASTING":
                ph = "COAST"

            est_err = (np.linalg.norm(tp_filtered - tp_oracle)
                       if tp_filtered is not None else -1.0)

            if (self.use_vision and est_err > 50.0
                    and self.state != "TRACKING"):
                self.get_logger().warn(
                    f"EKF DIVERGED (err={est_err:.0f}m) — forcing reset")
                self.ekf.reset()
                self.nav.reset()

            mode_tag = "V" if self.use_vision else "O"
            extra = (f"vis={self.vision_count} acc={self.acoustic_count} "
                     f"alt={ip[2]:.0f}") if self.use_vision else ""
            self.get_logger().info(
                f"t={el_t:5.1f}s [{mode_tag}] Sep:{sep_true:5.1f}m "
                f"EstErr:{est_err:.2f}m Vc={self.nav.Vc:+.1f} "
                f"ZEM={self.nav.zem:.1f} [{ph}] WP:{self.wp} "
                f"H:{self.hits} {extra}"
            )
        self.lp += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision', action='store_true',
                        help='use /spear/target_estimate as EKF input')
    args = parser.parse_args()

    print("=" * 60)
    mode = "VISION (closed-loop)" if args.vision else "ORACLE (baseline)"
    print(f"  SPEAR v3.0 — SENSOR FUSION (vision + acoustic) — mode: {mode}")
    print("=" * 60)

    rclpy.init()
    node = SpearNodeV2(use_vision=args.vision)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        el = time.time() - node.t0
        rate = node.hits / max(el, 1) * 60
        print(f"\n  {node.hits} hits in {el:.0f}s ({rate:.1f}/min)")
        if args.vision:
            print(f"  Vision detections: {node.vision_count}")
            print(f"  Acoustic detections: {node.acoustic_count}")

        node.shutdown_requested = True
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
