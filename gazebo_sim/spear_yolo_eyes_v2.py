#!/usr/bin/env python3
"""
SPEAR YOLO Eyes v2 — full perception pipeline.

Stage 1: Subscribes to camera, runs YOLO, displays annotated feed.
Stage 2: Subscribes to interceptor pose, runs the full math chain
         (bbox -> camera -> body -> world) on every detection, and
         publishes the world-frame target position estimate.

Publishes:
    /spear/target_estimate    geometry_msgs/PoseStamped
                              (world-frame target position; orientation
                               unused for now, set to identity)

Subscribes:
    /interceptor/camera                              sensor_msgs/Image
    /world/spear_intercept/dynamic_pose/info         tf2_msgs/TFMessage
"""
import sys
import math
import time
import cv2
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge

from ultralytics import YOLO

# ============ CONFIG ============
MODEL_PATH = Path.home() / "autonomous-uav-research" / "spear_drone_v3.pt"
CAMERA_TOPIC = "/interceptor/camera"
POSE_TOPIC = "/world/spear_intercept/dynamic_pose/info"
TARGET_ESTIMATE_TOPIC = "/spear/target_estimate"
CONF_THRESHOLD = 0.05
WINDOW_NAME = "SPEAR Interceptor View [YOLO v2]"
HOLD_DURATION = 0.5

CAPTURE_DIR = Path.home() / "autonomous-uav-research" / "gazebo_sim" / "baselines" / "yolo_captures"

# Camera intrinsics derived from SDF (HFOV=1.39626 rad, 640x480)
HFOV = 1.39626
IMG_W = 640
IMG_H = 480
FX = (IMG_W / 2) / math.tan(HFOV / 2)
FY = FX
CX = IMG_W / 2
CY = IMG_H / 2

# Target physical width in meters (from SDF box: 0.4 0.4 0.1)
TARGET_REAL_WIDTH = 0.4

# Camera mount on body (from SDF camera pose 0.25 0 -0.02 0 0 0)
T_CAM_IN_BODY = np.array([0.25, 0.0, -0.02])

# Camera frame -> Body frame rotation (constant, frame convention only)
R_CAM_TO_BODY = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0, -1,  0],
], dtype=float)


# ============ MATH HELPERS ============
def bbox_to_camera_position(x1, y1, x2, y2):
    """Bbox in pixels -> 3D target position in camera frame (meters)."""
    bbox_w = abs(x2 - x1)
    if bbox_w < 1.0:
        return None
    # Bearing
    u = (x1 + x2) / 2.0
    v = (y1 + y2) / 2.0
    x = (u - CX) / FX
    y = (v - CY) / FY
    z = 1.0
    ray = np.array([x, y, z])
    ray = ray / np.linalg.norm(ray)
    # Range
    rng = FX * TARGET_REAL_WIDTH / bbox_w
    return ray * rng


def cam_to_body(p_cam):
    return R_CAM_TO_BODY @ np.asarray(p_cam, dtype=float) + T_CAM_IN_BODY


def quat_to_rotation_matrix(qw, qx, qy, qz):
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-10:
        return np.eye(3)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def body_to_world(p_body, int_pos, int_quat):
    R = quat_to_rotation_matrix(*int_quat)
    return R @ np.asarray(p_body, dtype=float) + np.asarray(int_pos, dtype=float)


# ============ NODE ============
class SpearEyesV2(Node):
    def __init__(self):
        super().__init__("spear_eyes_v2")

        self.get_logger().info(f"Loading YOLO model: {MODEL_PATH}")
        if not MODEL_PATH.exists():
            self.get_logger().error(f"Model not found at {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(str(MODEL_PATH))
        self.get_logger().info(f"Model loaded. Classes: {self.model.names}")
        self.get_logger().info(
            f"Camera intrinsics: fx=fy={FX:.2f} cx={CX} cy={CY}"
        )

        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()

        # Interceptor pose state (updated by pose callback)
        self.interceptor_pos = None      # np.array([x, y, z]) or None
        self.interceptor_quat = None     # np.array([w, x, y, z]) or None
        self.pose_msgs_received = 0

        # Frame stats
        self.frame_count = 0
        self.detection_count = 0
        self.publish_count = 0
        self.last_log_time = time.time()
        self.last_fps_frames = 0
        self.held_boxes = []
        self.held_until = 0.0
        self.best_conf_seen = 0.0
        self.captures_saved = 0

        # Subscribers
        self.sub_image = self.create_subscription(
            Image, CAMERA_TOPIC, self.on_image, 10)
        self.sub_pose = self.create_subscription(
            TFMessage, POSE_TOPIC, self.on_pose, 10)

        # Publisher
        self.pub_target = self.create_publisher(
            PoseStamped, TARGET_ESTIMATE_TOPIC, 10)

        self.get_logger().info(f"Subscribed: {CAMERA_TOPIC}")
        self.get_logger().info(f"Subscribed: {POSE_TOPIC}")
        self.get_logger().info(f"Publishing: {TARGET_ESTIMATE_TOPIC}")
        self.get_logger().info("Waiting for first frame and first pose...")

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 800, 600)

    def on_pose(self, msg: TFMessage):
        """Extract interceptor pose from the world TF message."""
        self.pose_msgs_received += 1
        for tf in msg.transforms:
            # The interceptor entity is named "interceptor" in the SDF
            if tf.child_frame_id == "interceptor":
                t = tf.transform.translation
                r = tf.transform.rotation
                self.interceptor_pos = np.array([t.x, t.y, t.z])
                self.interceptor_quat = np.array([r.w, r.x, r.y, r.z])
                return

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        if self.frame_count == 0:
            self.get_logger().info(
                f"First frame received: {frame.shape[1]}x{frame.shape[0]}")
        self.frame_count += 1
        now = time.time()

        # Run YOLO
        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)
        new_boxes = []
        max_conf = 0.0
        best_box = None
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.model.names.get(cls_id, str(cls_id))
                label = f"{cls_name} {conf:.2f}"
                new_boxes.append((x1, y1, x2, y2, conf, label))
                if conf > max_conf:
                    max_conf = conf
                    best_box = (x1, y1, x2, y2)
                self.detection_count += 1

        # If we got a detection AND we have a current interceptor pose,
        # run the math chain and publish.
        published_this_frame = False
        target_world = None
        if best_box is not None and self.interceptor_pos is not None:
            x1, y1, x2, y2 = best_box
            p_cam = bbox_to_camera_position(x1, y1, x2, y2)
            if p_cam is not None:
                p_body = cam_to_body(p_cam)
                target_world = body_to_world(
                    p_body, self.interceptor_pos, self.interceptor_quat)
                self.publish_target_estimate(target_world, msg.header.stamp)
                published_this_frame = True
                self.publish_count += 1

        # Held-detection visualization (unchanged from v1)
        if new_boxes:
            self.held_boxes = new_boxes
            self.held_until = now + HOLD_DURATION
        if max_conf > self.best_conf_seen and max_conf > 0:
            self.best_conf_seen = max_conf

        if now < self.held_until:
            draw_boxes = self.held_boxes
            box_color = (0, 255, 0) if new_boxes else (0, 200, 200)
        else:
            draw_boxes = []
            self.held_boxes = []
            box_color = (0, 255, 0)

        annotated = self._draw_boxes(frame.copy(), draw_boxes, color=box_color)

        # Periodic stats
        elapsed = now - self.last_log_time
        if elapsed >= 1.0:
            fps = (self.frame_count - self.last_fps_frames) / elapsed
            self.last_log_time = now
            self.last_fps_frames = self.frame_count
            pose_status = "OK" if self.interceptor_pos is not None else "MISSING"
            self.get_logger().info(
                f"frames={self.frame_count} det={self.detection_count} "
                f"published={self.publish_count} fps={fps:.1f} "
                f"best_conf={self.best_conf_seen:.2f} pose={pose_status}"
            )

        # HUD
        hud1 = (f"frame {self.frame_count}  live_det {len(new_boxes)}  "
                f"published {self.publish_count}")
        hud2 = (f"best_conf {self.best_conf_seen:.2f}  "
                f"pose {'OK' if self.interceptor_pos is not None else 'WAIT'}")
        if target_world is not None:
            hud3 = (f"target_world: ({target_world[0]:.1f}, "
                    f"{target_world[1]:.1f}, {target_world[2]:.1f})")
        else:
            hud3 = "target_world: --"
        cv2.putText(annotated, hud1, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(annotated, hud2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        cv2.putText(annotated, hud3, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        cv2.imshow(WINDOW_NAME, annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rclpy.shutdown()

    def publish_target_estimate(self, p_world, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.pose.position.x = float(p_world[0])
        msg.pose.position.y = float(p_world[1])
        msg.pose.position.z = float(p_world[2])
        msg.pose.orientation.w = 1.0  # identity, orientation unused
        self.pub_target.publish(msg)

    def _draw_boxes(self, img, boxes, color=(0, 255, 0)):
        for (x1, y1, x2, y2, conf, label) in boxes:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.drawMarker(img, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 18, 2)
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(img, (x1, y1 - th - 6),
                          (x1 + tw + 4, y1), color, -1)
            cv2.putText(img, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
        return img


def main():
    rclpy.init()
    node = SpearEyesV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
