#!/usr/bin/env python3
"""
SPEAR Acoustic Cuer v2 — simulated mic array, WORLD FRAME output.

CHANGE FROM v1:
  - Now publishes target bearing in WORLD frame, not body frame.
    Removes the broken body-frame conversion that caused oscillation.

Publishes:
    /spear/acoustic_bearing    geometry_msgs/Vector3
        x = world_az (rad, atan2(dy, dx))
        y = world_el (rad, atan2(dz, horiz))
        z = signal strength (0-1)
"""
import sys
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from tf2_msgs.msg import TFMessage

ACOUSTIC_MAX_RANGE = 300.0
ACOUSTIC_NOISE_DEG = 5.0
ACOUSTIC_RATE_HZ = 5.0
ACOUSTIC_DROPOUT_PROB = 0.10
SIGNAL_FALLOFF_REF = 30.0

OUTPUT_TOPIC = "/spear/acoustic_bearing"
POSE_TOPIC = "/world/spear_intercept/dynamic_pose/info"


class SpearAcousticCuer(Node):
    def __init__(self):
        super().__init__("spear_acoustic_cuer")
        self.int_pos = None
        self.tgt_pos = None

        self.create_subscription(TFMessage, POSE_TOPIC, self.on_pose, 10)
        self.pub = self.create_publisher(Vector3, OUTPUT_TOPIC, 10)
        self.create_timer(1.0 / ACOUSTIC_RATE_HZ, self.tx)

        self.tx_count = 0
        self.dropout_count = 0
        self.out_of_range_count = 0
        self.last_log = time.time()

        self.get_logger().info("=" * 60)
        self.get_logger().info("SPEAR Acoustic Cuer v2 — WORLD FRAME bearing")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"Max range: {ACOUSTIC_MAX_RANGE}m  Noise: ±{ACOUSTIC_NOISE_DEG}°  "
            f"Rate: {ACOUSTIC_RATE_HZ}Hz  Dropout: {ACOUSTIC_DROPOUT_PROB*100:.0f}%")
        self.get_logger().info(f"Publishing: {OUTPUT_TOPIC}")

    def on_pose(self, msg: TFMessage):
        for tf in msg.transforms:
            n = tf.child_frame_id
            t = tf.transform.translation
            if n == "interceptor":
                self.int_pos = np.array([t.x, t.y, t.z])
            elif n == "target":
                self.tgt_pos = np.array([t.x, t.y, t.z])

    def tx(self):
        if self.int_pos is None or self.tgt_pos is None:
            return

        rel = self.tgt_pos - self.int_pos
        rng = np.linalg.norm(rel)

        if rng > ACOUSTIC_MAX_RANGE:
            self.out_of_range_count += 1
            self._maybe_log(rng)
            return

        if np.random.random() < ACOUSTIC_DROPOUT_PROB:
            self.dropout_count += 1
            self._maybe_log(rng)
            return

        x, y, z = rel
        az_true = math.atan2(y, x)
        horiz = math.sqrt(x*x + y*y)
        el_true = math.atan2(z, horiz)

        noise_rad = math.radians(ACOUSTIC_NOISE_DEG)
        az_noisy = az_true + np.random.normal(0, noise_rad)
        el_noisy = el_true + np.random.normal(0, noise_rad)

        signal = 1.0 / (1.0 + (rng / SIGNAL_FALLOFF_REF) ** 2)
        signal = max(0.0, min(1.0, signal))

        m = Vector3()
        m.x = float(az_noisy)
        m.y = float(el_noisy)
        m.z = float(signal)
        self.pub.publish(m)

        self.tx_count += 1
        self._maybe_log(rng)

    def _maybe_log(self, rng):
        now = time.time()
        if now - self.last_log >= 2.0:
            self.last_log = now
            self.get_logger().info(
                f"tx={self.tx_count}  drop={self.dropout_count}  "
                f"oor={self.out_of_range_count}  rng={rng:.1f}m")


def main():
    rclpy.init()
    node = SpearAcousticCuer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
