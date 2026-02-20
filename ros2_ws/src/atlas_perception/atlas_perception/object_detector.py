"""
Object Detector Node
====================
Subscribes to camera images and runs YOLO object detection.
Reuses existing Atlas YOLO model (yolov8n.pt at project root).
Publishes detection results as PerceptionResult messages.

Bridges to: modules/humanoid/vision/
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import os


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("atlas.perception.yolo_model", "yolov8n")

        self._model_name = self.get_parameter("atlas.perception.yolo_model").value
        self._model = None
        self._frame_skip = 3  # process every Nth frame
        self._frame_count = 0

        # Try to load YOLO model
        self._load_model()

        # Subscriber
        self.create_subscription(Image, "/atlas/camera/head/rgb", self._image_cb, 10)

        # Publishers
        self.detections_pub = self.create_publisher(String, "/atlas/perception/detections", 10)

        self.get_logger().info(
            f"Object detector started: model={self._model_name}, "
            f"loaded={'YES' if self._model else 'NO (will publish empty)'}"
        )

    def _load_model(self):
        """Attempt to load YOLO model from existing Atlas project."""
        try:
            from ultralytics import YOLO
            # Check common locations for the model
            candidates = [
                os.path.join("C:", os.sep, "ATLAS_PUSH", "yolov8n.pt"),
                os.path.join("C:", os.sep, "ATLAS_PUSH", "models", "yolov8n.pt"),
                f"{self._model_name}.pt",
            ]
            for path in candidates:
                if os.path.exists(path):
                    self._model = YOLO(path)
                    self.get_logger().info(f"YOLO model loaded from {path}")
                    return
            # Try downloading
            self._model = YOLO(f"{self._model_name}.pt")
            self.get_logger().info(f"YOLO model downloaded: {self._model_name}")
        except Exception as e:
            self.get_logger().warn(f"Could not load YOLO model: {e}")
            self._model = None

    def _image_cb(self, msg: Image):
        self._frame_count += 1
        if self._frame_count % self._frame_skip != 0:
            return

        detections = []

        if self._model is not None:
            try:
                import numpy as np
                # Convert ROS Image to numpy
                if msg.encoding in ("rgb8", "bgr8"):
                    channels = 3
                    dtype = np.uint8
                else:
                    # Skip unsupported encodings
                    return

                img = np.frombuffer(msg.data, dtype=dtype).reshape(
                    msg.height, msg.width, channels
                )

                results = self._model(img, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        cls_name = r.names.get(cls_id, f"class_{cls_id}")
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        detections.append({
                            "class": cls_name,
                            "confidence": round(conf, 3),
                            "bbox": [round(v, 1) for v in [x1, y1, x2, y2]],
                        })
            except Exception as e:
                self.get_logger().warn(f"Detection error: {e}", throttle_duration_sec=5.0)

        # Publish detections as JSON
        det_msg = String()
        det_msg.data = json.dumps({
            "frame": self._frame_count,
            "count": len(detections),
            "detections": detections,
        })
        self.detections_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
