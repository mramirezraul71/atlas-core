"""
SLAM Node (placeholder)
=======================
Visual/LiDAR SLAM for Atlas robot localization and mapping.
This is a placeholder that publishes identity pose.
Replace with real SLAM (ORB-SLAM3, rtabmap, etc.) when ready.

Bridges to: modules/humanoid/navigation/
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import math
import time


class SlamNode(Node):
    def __init__(self):
        super().__init__("slam_node")

        self.declare_parameter("atlas.perception.slam_enabled", False)
        self.slam_enabled = self.get_parameter("atlas.perception.slam_enabled").value

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/atlas/slam/pose", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/atlas/slam/map", 10)
        self.status_pub = self.create_publisher(Bool, "/atlas/slam/active", 10)

        # Simulated pose state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        if self.slam_enabled:
            self.timer = self.create_timer(0.1, self._publish_pose)  # 10Hz
            self.map_timer = self.create_timer(2.0, self._publish_map)  # 0.5Hz
            self.get_logger().info("SLAM node started (placeholder mode)")
        else:
            self.timer = self.create_timer(1.0, self._publish_status_only)
            self.get_logger().info("SLAM node started (DISABLED, publishing status only)")

    def _publish_status_only(self):
        msg = Bool()
        msg.data = False
        self.status_pub.publish(msg)

    def _publish_pose(self):
        # Status
        status = Bool()
        status.data = True
        self.status_pub.publish(status)

        # Pose (identity / origin for now)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(self._theta / 2.0)
        msg.pose.orientation.w = math.cos(self._theta / 2.0)
        self.pose_pub.publish(msg)

    def _publish_map(self):
        """Publish a small empty occupancy grid as placeholder."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.05  # 5cm per cell
        msg.info.width = 100        # 5m x 5m
        msg.info.height = 100
        msg.info.origin.position.x = -2.5
        msg.info.origin.position.y = -2.5
        msg.info.origin.position.z = 0.0
        msg.data = [-1] * (100 * 100)  # unknown
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
