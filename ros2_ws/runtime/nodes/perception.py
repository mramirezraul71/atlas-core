"""
Perception nodes for Atlas spine runtime (lite mode).
Vision, Object Detection (YOLO bridge), SLAM placeholder.
"""
import json
import os
import time
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import atlas_ros2_lite as rclpy
from atlas_ros2_lite import Node, make_string, make_bool, make_pose_stamped, make_header


class VisionNode(Node):
    def __init__(self, fps=5):
        super().__init__("vision_node")
        self._frame_count = 0
        self.status_pub = self.create_publisher(None, "/atlas/camera/status", 10)
        self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info(f"Vision node started (SIM, {fps}fps status only)")

    def _tick(self):
        self._frame_count += 1
        self.status_pub.publish(make_string(json.dumps({
            "frame": self._frame_count,
            "cameras": ["head_rgb", "chest_depth"],
            "mode": "simulation",
        })))


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self._model = None
        self._frame_count = 0

        # Try loading YOLO from existing Atlas
        try:
            from ultralytics import YOLO
            model_path = os.path.join("C:", os.sep, "ATLAS_PUSH", "yolov8n.pt")
            if os.path.exists(model_path):
                self._model = YOLO(model_path)
                self.get_logger().info(f"YOLO loaded from {model_path}")
            else:
                self.get_logger().info("YOLO model not found, detector in standby")
        except ImportError:
            self.get_logger().info("ultralytics not available, detector in standby")

        self.det_pub = self.create_publisher(None, "/atlas/perception/detections", 10)
        self.create_timer(1.0, self._tick)  # 1Hz status

    def _tick(self):
        self._frame_count += 1
        self.det_pub.publish(make_string(json.dumps({
            "frame": self._frame_count,
            "count": 0,
            "detections": [],
            "model_loaded": self._model is not None,
        })))


class SlamNode(Node):
    def __init__(self):
        super().__init__("slam_node")
        self.pose_pub = self.create_publisher(None, "/atlas/slam/pose", 10)
        self.status_pub = self.create_publisher(None, "/atlas/slam/active", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("SLAM node started (placeholder, publishing origin pose)")

    def _tick(self):
        self.status_pub.publish(make_bool(False))
        self.pose_pub.publish(make_pose_stamped(
            header=make_header("map"),
            position={"x": 0.0, "y": 0.0, "z": 0.0},
        ))
