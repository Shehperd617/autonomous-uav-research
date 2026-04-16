#!/usr/bin/env python3
"""
SPEAR Bridge v3.1 — SINGLE-PASS INTERCEPT.

One shot. Launch on noisy bearing, close using sensor fusion
(acoustic → vision → EKF → ProNav), hit or miss, done.

No re-engagement, no rearm, no patrol. This is how real
interceptors work and it's the cleanest scenario for
Monte Carlo evaluation.

STATE MACHINE:
  LAUNCH    → fly initial bearing (max 3s or until vision acquired)
  TRACKING  → vision fresh, full-speed ProNav
  COASTING  → vision stale 0.5-1.5s, half-speed ProNav on EKF
  SEARCHING → vision lost >1.5s, acoustic world-bearing chase
  HIT       → sep < HIT_DIST, engagement success, stop
  MISS      → timeout or diverged, engagement failure, stop

USAGE:
  python3 spear_ros2_bridge_v3.py --vision
  python3 spear_ros2_bridge_v3.py --vision --timeout 30
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

# ============ SCENARIO CONFIG ============
WAYPOINTS = [
    (50, 25, 20), (55, 20, 21), (52, 28, 19),
    (48, 22, 21), (53, 26, 20), (50, 18, 20),
    (47, 24, 21), (54, 22, 19), (51, 27, 20),
]
TGT_SPD = 4.0
TGT_SPD_FAST = 5.0
WP_THR = 3.0

# ============ LAUNCH CONFIG ============
LAUNCH_NOISE_DEG = 7.0
LAUNCH_SPEED = 22.0
LAUNCH_MAX_TIME = 3.0

# ============ INTERCEPTOR CONFIG ============
INT_SPD = 22.0
HIT_DIST = 3.0
TERMINAL_R = 12.0
CONTROL_HZ = 20
LEAD_GAIN = 0.8
N_PN = 4.5

# ============ EKF CONFIG ============
EKF_Q_POS = 0.5
EKF_Q_VEL = 25.0
EKF_R_BASE = 0.05
EKF_VISION_NOISE_STD = 1.0
EKF_ORACLE_NOISE_STD = 0.05

# ============ STATE THRESHOLDS ============
STALE_THRESHOLD = 0.5
LOST_THRESHOLD = 1.5

# ============ SEARCH CONFIG ============
SEARCH_CHASE_SPEED = 22.0
SEARCH_YAW_RATE = 0.6

# ============ ACOUSTIC CONFIG ============
ACOUSTIC_STALE_THRESHOLD = 1.0
ACOUSTIC_MIN_SIGNAL = 0.05

# ============ SAFETY ============
ALT_MIN = 10.0
ALT_MAX = 30.0

# ============ SINGLE-PASS ============
MISS_TIMEOUT = 60.0
MISS_RANGE = 200.0


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
        self.H[0, 0] = 1; self.H[1, 1] = 1; self.H[2, 2] = 1
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
        F[0, 3] = dt; F[1, 4] = dt; F[2, 5] = dt
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
        self.P = (np.eye(6) - K @ self.H) @ self.P

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
        self.__init__()

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
                        self._save(lh, t, tp)
                        return (lead_los / ll) * INT_SPD
                    self._save(lh, t, tp)
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
                self._save(lh, t, tp)
                return v

        self._save(lh, t, tp)
        return lh * INT_SPD

    def _save(self, lh, t, tp):
        self.prev_los = lh.copy()
        self.prev_R = self.R
        self.prev_t = t
        self.prev_tp = tp.copy()


# ============ NODE ============
class SpearNodeV3(Node):
    def __init__(self, use_vision=False, timeout=MISS_TIMEOUT):
        super().__init__('spear_bridge_v3')
        self.use_vision = use_vision
        self.timeout = timeout
        self.shutdown_requested = False
        self.engagement_over = False
        self.result = None
        self.hit_time = None
        self.min_sep = 999.0
        self.ever_tracked = False

        self.int_pub = self.create_publisher(
            Twist, '/model/interceptor/cmd_vel', 10)
        self.tgt_pub = self.create_publisher(
            Twist, '/model/target/cmd_vel', 10)

        self.int_pos = None
        self.tgt_pos_oracle = None
        self.tgt_pos_vision = None
        self.vision_count = 0
        self.last_vision_t = None

        self.acoustic_az_world = None
        self.acoustic_el_world = None
        self.acoustic_signal = 0.0
        self.acoustic_count = 0
        self.last_acoustic_t = None

        meas_noise = (EKF_VISION_NOISE_STD if use_vision
                      else EKF_ORACLE_NOISE_STD)
        self.ekf = TargetEKF(meas_noise_std=meas_noise)
        self.nav = ProNav3D()

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

        self.wp = 0
        self.t0 = time.time()
        self.lp = 0
        self.pose_count = 0
        self.last_iv = np.zeros(3)
        self.last_iv_angular = np.zeros(3)
        self.last_tv = np.zeros(3)

        self.state = "LAUNCH"
        self.launch_bearing = None
        self.launch_t = None

        mode = "VISION" if use_vision else "ORACLE"
        self.get_logger().info(
            f"SPEAR v3.1 SINGLE-PASS — {CONTROL_HZ}Hz — {mode}")
        self.get_logger().info(
            f"Launch noise: +/-{LAUNCH_NOISE_DEG} deg  "
            f"Timeout: {timeout}s  Hit dist: {HIT_DIST}m")
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
        az = self.acoustic_az_world
        el = self.acoustic_el_world
        if az is None or el is None:
            return None
        cx = math.cos(el) * math.cos(az)
        cy = math.cos(el) * math.sin(az)
        cz = math.sin(el)
        return np.array([cx, cy, cz]) * speed

    def apply_altitude_cap(self, ip, v):
        v = v.copy()
        if ip[2] >= ALT_MAX and v[2] > 0:
            v[2] = 0
        elif ip[2] <= ALT_MIN and v[2] < 0:
            v[2] = 0
        if ip[2] > ALT_MAX + 5:
            v[2] = min(v[2], -5.0)
        elif ip[2] < ALT_MIN - 2:
            v[2] = max(v[2], 5.0)
        return v

    def compute_launch_bearing(self, ip, tp):
        true_dir = tp - ip
        true_dist = np.linalg.norm(true_dir)
        if true_dist < 0.1:
            return np.array([1.0, 0.0, 0.0])
        true_unit = true_dir / true_dist
        noise_rad = math.radians(LAUNCH_NOISE_DEG)
        az_true = math.atan2(true_dir[1], true_dir[0])
        el_true = math.atan2(true_dir[2],
                             math.sqrt(true_dir[0]**2 + true_dir[1]**2))
        az_noisy = az_true + np.random.normal(0, noise_rad)
        el_noisy = el_true + np.random.normal(0, noise_rad)
        noisy_unit = np.array([
            math.cos(el_noisy) * math.cos(az_noisy),
            math.cos(el_noisy) * math.sin(az_noisy),
            math.sin(el_noisy)])
        err_deg = math.degrees(math.acos(
            np.clip(np.dot(true_unit, noisy_unit), -1, 1)))
        self.get_logger().info(
            f"LAUNCH BEARING: err={err_deg:.1f} deg  range={true_dist:.1f}m")
        return noisy_unit

    def get_target_measurement(self, t):
        if not self.use_vision:
            return self.tgt_pos_oracle, True
        if self.tgt_pos_vision is None or self.last_vision_t is None:
            return None, False
        if t - self.last_vision_t > STALE_THRESHOLD:
            return None, False
        return self.tgt_pos_vision, True

    def compute_search_velocity(self, ip, t):
        if self.acoustic_fresh(t) and self.acoustic_az_world is not None:
            iv = self.acoustic_world_velocity(SEARCH_CHASE_SPEED)
            if iv is not None:
                return iv, "ACC"
        if self.launch_bearing is not None:
            return self.launch_bearing * SEARCH_CHASE_SPEED, "LB"
        return np.zeros(3), "NONE"

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

    def end_engagement(self, result, sep, el_t):
        self.engagement_over = True
        self.result = result
        self.last_iv = np.zeros(3)
        self.last_iv_angular = np.zeros(3)
        if result == "HIT":
            self.hit_time = el_t
        lvl = self.get_logger().info if result == "HIT" else self.get_logger().warn
        lvl(f"*** {result} *** t={el_t:.1f}s sep={sep:.1f}m "
            f"vis={self.vision_count} acc={self.acoustic_count} "
            f"min_sep={self.min_sep:.1f}m")

    def tick(self):
        el_t = time.time() - self.t0

        if self.engagement_over:
            if self.lp % 40 == 0:
                self.get_logger().info(
                    f"Engagement ended: {self.result} (Ctrl+C to exit)")
            self.lp += 1
            return

        if self.int_pos is None or self.tgt_pos_oracle is None:
            if self.lp % 20 == 0:
                self.get_logger().warn(
                    f"Waiting for poses... ({self.pose_count})")
            self.lp += 1
            return

        ip = self.int_pos.copy()
        tp_oracle = self.tgt_pos_oracle.copy()

        if self.launch_bearing is None:
            self.launch_bearing = self.compute_launch_bearing(ip, tp_oracle)
            self.launch_t = el_t

        # Target waypoint chase
        w = np.array(WAYPOINTS[self.wp])
        d = w - tp_oracle
        dn = np.linalg.norm(d)
        if dn < WP_THR:
            self.wp = (self.wp + 1) % len(WAYPOINTS)
            w = np.array(WAYPOINTS[self.wp])
            d = w - tp_oracle
            dn = np.linalg.norm(d)
        s = TGT_SPD if self.wp < 3 else TGT_SPD_FAST
        self.last_tv = (d / dn) * s if dn > 0.1 else np.zeros(3)

        # Separation
        sep = np.linalg.norm(tp_oracle - ip)
        self.min_sep = min(self.min_sep, sep)

        # End conditions
        if sep < HIT_DIST:
            self.end_engagement("HIT", sep, el_t)
            return
        if el_t > self.timeout:
            self.end_engagement("MISS-TIMEOUT", sep, el_t)
            return
        if self.ever_tracked and sep > MISS_RANGE:
            self.end_engagement("MISS-DIVERGED", sep, el_t)
            return

        # EKF measurement
        meas, has_new = self.get_target_measurement(el_t)

        # State transitions
        old_state = self.state
        if self.state == "LAUNCH":
            if has_new:
                self.state = "TRACKING"
            elif self.launch_t and (el_t - self.launch_t) > LAUNCH_MAX_TIME:
                self.state = "SEARCHING"
        elif self.state == "TRACKING":
            if not has_new:
                if self.last_vision_t is None:
                    self.state = "SEARCHING"
                else:
                    age = el_t - self.last_vision_t
                    if age > LOST_THRESHOLD:
                        self.state = "SEARCHING"
                    elif age > STALE_THRESHOLD:
                        self.state = "COASTING"
        elif self.state == "COASTING":
            if has_new:
                self.state = "TRACKING"
            elif self.last_vision_t and (el_t - self.last_vision_t) > LOST_THRESHOLD:
                self.state = "SEARCHING"
        elif self.state == "SEARCHING":
            if has_new:
                self.state = "TRACKING"

        if old_state != self.state:
            age = (el_t - self.last_vision_t) if self.last_vision_t else -1
            self.get_logger().warn(
                f"State: {old_state} -> {self.state}  "
                f"(age={age:.2f}s  vis={self.vision_count} "
                f"acc={self.acoustic_count})")
            if self.state == "TRACKING" and old_state != "COASTING":
                self.get_logger().warn("EKF RESET — fresh vision lock")
                self.ekf.reset()
                self.nav.reset()

        if self.state == "TRACKING":
            self.ever_tracked = True

        # EKF step
        if has_new:
            tp_f, tv_f = self.ekf.step_with_measurement(meas, el_t)
        else:
            tp_f, tv_f = self.ekf.step_predict_only(el_t)

        # EKF divergence gate
        est_err = np.linalg.norm(tp_f - tp_oracle) if tp_f is not None else -1
        if self.use_vision and est_err > 50.0 and self.state != "TRACKING":
            self.get_logger().warn(f"EKF DIVERGED (err={est_err:.0f}m) — reset")
            self.ekf.reset()
            self.nav.reset()
            tp_f, tv_f = None, None
            est_err = -1

        # Compute velocity
        if self.state == "LAUNCH":
            iv = self.launch_bearing * LAUNCH_SPEED
        elif self.state == "SEARCHING":
            iv, src = self.compute_search_velocity(ip, el_t)
            self._search_src = src
            self.last_iv_angular = np.array([0.0, 0.0, SEARCH_YAW_RATE])
        elif tp_f is not None:
            iv = self.nav.compute(ip, tp_f, el_t, ekf_tgt_vel=tv_f)
            if self.state == "COASTING":
                iv = iv * 0.5
            if self.state == "TRACKING" and sep > 80.0:
                chase = tp_f - ip
                cn = np.linalg.norm(chase)
                if cn > 1.0:
                    iv = (chase / cn) * INT_SPD
            sp = np.linalg.norm(iv)
            if sp > INT_SPD * 1.5:
                iv = iv / sp * INT_SPD * 1.5
            self.last_iv_angular = np.zeros(3)
        else:
            iv = np.zeros(3)
            self.last_iv_angular = np.zeros(3)

        iv = self.apply_altitude_cap(ip, iv)
        self.last_iv = iv

        # Status log
        if self.lp % 10 == 0:
            if self.state == "LAUNCH":
                ph = "LAUNCH"
            elif self.state == "SEARCHING":
                ph = f"HUNT-{getattr(self, '_search_src', '?')}"
            elif self.state == "COASTING":
                ph = "COAST"
            else:
                ph = "PN" if sep >= TERMINAL_R else "TM"

            mode_tag = "V" if self.use_vision else "O"
            self.get_logger().info(
                f"t={el_t:5.1f}s [{mode_tag}] Sep:{sep:5.1f}m "
                f"EstErr:{est_err:.1f}m Vc={self.nav.Vc:+.1f} "
                f"[{ph}] vis={self.vision_count} "
                f"acc={self.acoustic_count} alt={ip[2]:.0f}"
            )
        self.lp += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision', action='store_true')
    parser.add_argument('--timeout', type=float, default=MISS_TIMEOUT)
    args = parser.parse_args()

    mode = "VISION" if args.vision else "ORACLE"
    print("=" * 50)
    print(f"  SPEAR v3.1 SINGLE-PASS — {mode}")
    print(f"  Timeout: {args.timeout}s")
    print("=" * 50)

    rclpy.init()
    node = SpearNodeV3(use_vision=args.vision, timeout=args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        el = time.time() - node.t0
        result = node.result or "INTERRUPTED"
        print(f"\n  Result: {result}")
        print(f"  Time: {el:.1f}s")
        if node.hit_time:
            print(f"  Time-to-hit: {node.hit_time:.1f}s")
        print(f"  Min separation: {node.min_sep:.1f}m")
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
