"""
Path Planner Node
=================
Navigation path planning for Atlas humanoid.
Subscribes to SLAM pose and occupancy grid, publishes waypoint paths.
Compatible with Nav2 interface for future integration.

Bridges to: modules/humanoid/navigation/
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
import json
import math


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        self.declare_parameter("atlas.planning.planner", "nav2")
        self.declare_parameter("atlas.planning.max_velocity", 0.5)
        self.declare_parameter("atlas.planning.obstacle_margin", 0.3)

        self.planner_type = self.get_parameter("atlas.planning.planner").value
        self.max_vel = self.get_parameter("atlas.planning.max_velocity").value
        self.obstacle_margin = self.get_parameter("atlas.planning.obstacle_margin").value

        # State
        self._current_pose = None
        self._map = None

        # Subscribers
        self.create_subscription(PoseStamped, "/atlas/slam/pose", self._pose_cb, 10)
        self.create_subscription(OccupancyGrid, "/atlas/slam/map", self._map_cb, 10)
        self.create_subscription(PoseStamped, "/atlas/nav/goal", self._goal_cb, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, "/atlas/nav/path", 10)
        self.waypoints_pub = self.create_publisher(PoseArray, "/atlas/nav/waypoints", 10)
        self.status_pub = self.create_publisher(String, "/atlas/nav/status", 10)

        self.get_logger().info(
            f"Path planner started: type={self.planner_type}, "
            f"max_vel={self.max_vel}m/s, margin={self.obstacle_margin}m"
        )

    def _pose_cb(self, msg: PoseStamped):
        self._current_pose = msg

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _goal_cb(self, msg: PoseStamped):
        """Receive navigation goal and compute path."""
        if self._current_pose is None:
            self.get_logger().warn("No current pose available, cannot plan path")
            self._publish_status("error", "No pose available")
            return

        self.get_logger().info(
            f"Planning path to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        self._publish_status("planning", "Computing path")

        path = self._plan_straight_line(self._current_pose, msg)
        self.path_pub.publish(path)

        # Convert to waypoints
        waypoints = PoseArray()
        waypoints.header = path.header
        waypoints.poses = [p.pose for p in path.poses]
        self.waypoints_pub.publish(waypoints)

        self._publish_status("ready", f"{len(path.poses)} waypoints")

    def _plan_straight_line(self, start: PoseStamped, goal: PoseStamped) -> Path:
        """Simple straight-line path with interpolated waypoints."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        sx = start.pose.position.x
        sy = start.pose.position.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y

        dist = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)
        step_size = 0.1  # 10cm between waypoints
        n_steps = max(2, int(dist / step_size))

        for i in range(n_steps + 1):
            t = i / n_steps
            wp = PoseStamped()
            wp.header = path.header
            wp.pose.position.x = sx + t * (gx - sx)
            wp.pose.position.y = sy + t * (gy - sy)
            wp.pose.position.z = 0.0

            # Face direction of travel
            heading = math.atan2(gy - sy, gx - sx)
            wp.pose.orientation.z = math.sin(heading / 2.0)
            wp.pose.orientation.w = math.cos(heading / 2.0)

            path.poses.append(wp)

        return path

    def _publish_status(self, status: str, detail: str):
        msg = String()
        msg.data = json.dumps({"status": status, "detail": detail})
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
