"""
Vision Node
===========
Camera driver and image publisher for Atlas head and chest cameras.
Bridges to existing Atlas vision modules (modules/humanoid/vision/).
In simulation, publishes synthetic images or reads from webcam.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np
import time


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        self.declare_parameter("atlas.perception.cameras", [])
        self.declare_parameter("atlas.hardware.enable_hardware", False)

        self.hw_enabled = self.get_parameter("atlas.hardware.enable_hardware").value

        # Publishers for each camera
        self.head_rgb_pub = self.create_publisher(Image, "/atlas/camera/head/rgb", 10)
        self.chest_depth_pub = self.create_publisher(Image, "/atlas/camera/chest/depth", 10)
        self.head_info_pub = self.create_publisher(CameraInfo, "/atlas/camera/head/info", 10)

        # Publish at configured FPS
        self.rgb_timer = self.create_timer(1.0 / 30.0, self._publish_head_rgb)
        self.depth_timer = self.create_timer(1.0 / 15.0, self._publish_chest_depth)

        self._frame_count = 0
        self._cap = None

        if self.hw_enabled:
            try:
                import cv2
                self._cap = cv2.VideoCapture(0)
                if self._cap.isOpened():
                    self.get_logger().info("Hardware camera opened (device 0)")
                else:
                    self._cap = None
                    self.get_logger().warn("Failed to open hardware camera, falling back to synthetic")
            except ImportError:
                self.get_logger().warn("cv2 not available, using synthetic images")

        self.get_logger().info(f"Vision node started (hw={'ON' if self.hw_enabled and self._cap else 'SIM'})")

    def _make_synthetic_rgb(self, width=640, height=480) -> Image:
        """Generate a synthetic RGB image with a gradient pattern."""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "head_camera_link"
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = width * 3

        # Simple gradient with frame counter variation
        t = self._frame_count % 256
        data = np.zeros((height, width, 3), dtype=np.uint8)
        data[:, :, 0] = np.linspace(t, (t + 128) % 256, width, dtype=np.uint8)
        data[:, :, 1] = np.linspace(0, 255, height, dtype=np.uint8).reshape(-1, 1)
        data[:, :, 2] = 128
        msg.data = data.tobytes()
        return msg

    def _make_synthetic_depth(self, width=320, height=240) -> Image:
        """Generate a synthetic depth image."""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "chest_depth_link"
        msg.height = height
        msg.width = width
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = width * 4

        # Flat floor at 1.0m with some noise
        data = np.ones((height, width), dtype=np.float32) * 1.0
        data += np.random.normal(0, 0.01, (height, width)).astype(np.float32)
        msg.data = data.tobytes()
        return msg

    def _publish_head_rgb(self):
        self._frame_count += 1

        if self._cap is not None:
            import cv2
            ret, frame = self._cap.read()
            if ret:
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "head_camera_link"
                msg.height, msg.width = frame.shape[:2]
                msg.encoding = "bgr8"
                msg.is_bigendian = False
                msg.step = frame.shape[1] * 3
                msg.data = frame.tobytes()
                self.head_rgb_pub.publish(msg)
                return

        self.head_rgb_pub.publish(self._make_synthetic_rgb())

    def _publish_chest_depth(self):
        self.chest_depth_pub.publish(self._make_synthetic_depth())

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
