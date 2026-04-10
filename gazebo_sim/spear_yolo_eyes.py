#!/usr/bin/env python3
"""
SPEAR YOLO Eyes — subscribes to the interceptor's simulated camera,
runs YOLO on every frame, draws bounding boxes, displays the live feed.

Stage 1 of the perception pipeline: 'the drone has eyes'.
v2: detection-hold overlay (boxes persist 500ms) + auto-screenshot on
best-confidence detection (saves to baselines/yolo_captures/).
"""
import sys
import time
import cv2
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

MODEL_PATH = Path.home() / "autonomous-uav-research" / "spear_drone_v3.pt"
CAMERA_TOPIC = "/interceptor/camera"
CONF_THRESHOLD = 0.05
WINDOW_NAME = "SPEAR Interceptor View [YOLO]"

# Detection hold: keep last detection visible for this many seconds
HOLD_DURATION = 0.5

# Auto-capture directory
CAPTURE_DIR = Path.home() / "autonomous-uav-research" / "gazebo_sim" / "baselines" / "yolo_captures"


class SpearEyes(Node):
    def __init__(self):
        super().__init__("spear_eyes")

        self.get_logger().info(f"Loading YOLO model: {MODEL_PATH}")
        if not MODEL_PATH.exists():
            self.get_logger().error(f"Model not found at {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(str(MODEL_PATH))
        self.get_logger().info(f"Model loaded. Classes: {self.model.names}")
        self.get_logger().info(f"Confidence threshold: {CONF_THRESHOLD}")

        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Auto-captures -> {CAPTURE_DIR}")

        self.bridge = CvBridge()
        self.frame_count = 0
        self.detection_count = 0
        self.last_log_time = time.time()
        self.last_fps_frames = 0

        # Detection hold state
        self.held_boxes = []        # list of (x1,y1,x2,y2,conf,label)
        self.held_until = 0.0       # wall-clock time when boxes expire

        # Best detection tracker
        self.best_conf_seen = 0.0
        self.captures_saved = 0

        self.sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.on_image, 10
        )
        self.get_logger().info(f"Subscribed to {CAMERA_TOPIC}")
        self.get_logger().info("Waiting for first frame...")

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 800, 600)

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        if self.frame_count == 0:
            self.get_logger().info(
                f"First frame received: {frame.shape[1]}x{frame.shape[0]}"
            )

        self.frame_count += 1
        now = time.time()

        # Run YOLO inference
        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)

        # Collect this frame's detections
        new_boxes = []
        max_conf_this_frame = 0.0
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
                if conf > max_conf_this_frame:
                    max_conf_this_frame = conf
                self.detection_count += 1

        # Update detection-hold buffer
        if new_boxes:
            self.held_boxes = new_boxes
            self.held_until = now + HOLD_DURATION

        # Auto-capture on new best confidence
        if max_conf_this_frame > self.best_conf_seen and max_conf_this_frame > 0.0:
            self.best_conf_seen = max_conf_this_frame
            annotated_for_save = self._draw_boxes(frame.copy(), new_boxes)
            self._save_capture(annotated_for_save, max_conf_this_frame)

        # Decide what to draw: live or held
        if now < self.held_until:
            draw_boxes = self.held_boxes
            box_color = (0, 255, 0) if new_boxes else (0, 200, 200)  # green live, yellow held
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
            self.get_logger().info(
                f"frames={self.frame_count}  detections={self.detection_count}  "
                f"fps={fps:.1f}  best_conf={self.best_conf_seen:.2f}  "
                f"saved={self.captures_saved}"
            )

        # HUD overlay
        hud1 = (f"frame {self.frame_count}  live_det {len(new_boxes)}  "
                f"held {len(self.held_boxes)}  conf>={CONF_THRESHOLD}")
        hud2 = f"best_conf {self.best_conf_seen:.2f}  saved {self.captures_saved}"
        cv2.putText(annotated, hud1, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(annotated, hud2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        cv2.imshow(WINDOW_NAME, annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Quit requested.")
            rclpy.shutdown()
        elif key == ord("s"):
            # Manual save trigger
            self._save_capture(annotated, self.best_conf_seen, manual=True)

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

    def _save_capture(self, img, conf, manual=False):
        tag = "manual" if manual else "best"
        fname = CAPTURE_DIR / f"{tag}_conf{conf:.2f}_frame{self.frame_count}.png"
        try:
            cv2.imwrite(str(fname), img)
            self.captures_saved += 1
            self.get_logger().info(f"SAVED: {fname.name}")
        except Exception as e:
            self.get_logger().warn(f"Save failed: {e}")


def main():
    rclpy.init()
    node = SpearEyes()
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
