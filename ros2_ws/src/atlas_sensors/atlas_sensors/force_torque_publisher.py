"""
Force-Torque Publisher Node
===========================
Publishes force-torque sensor data from feet (ground reaction forces).
Essential for balance control (IHMC-style impulse control).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import math
import time


class ForceTorquePublisher(Node):
    def __init__(self):
        super().__init__("force_torque_publisher")
        self.declare_parameter("atlas.hardware.sensor_rate_hz", 50)
        self.declare_parameter("atlas.hardware.enable_hardware", False)

        rate = self.get_parameter("atlas.hardware.sensor_rate_hz").value
        self.hw_enabled = self.get_parameter("atlas.hardware.enable_hardware").value

        self.pub_left = self.create_publisher(
            WrenchStamped, "/atlas/ft/left_foot", 10
        )
        self.pub_right = self.create_publisher(
            WrenchStamped, "/atlas/ft/right_foot", 10
        )
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.t0 = time.time()
        self.robot_mass_kg = 80.0  # estimated total mass
        self.get_logger().info(f"Force-torque publisher started at {rate}Hz")

    def _make_msg(self, frame_id: str, fz: float) -> WrenchStamped:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = fz
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        return msg

    def timer_callback(self):
        if self.hw_enabled:
            # TODO: read from real F/T sensors
            fz_left = self.robot_mass_kg * 9.81 / 2.0
            fz_right = fz_left
        else:
            # Simulation: weight distributed between feet with slight sway
            t = time.time() - self.t0
            total_weight = self.robot_mass_kg * 9.81
            sway = math.sin(t * 0.3) * 0.05  # 5% weight shift
            fz_left = total_weight * (0.5 + sway)
            fz_right = total_weight * (0.5 - sway)

        self.pub_left.publish(self._make_msg("l_foot_ft_link", fz_left))
        self.pub_right.publish(self._make_msg("r_foot_ft_link", fz_right))


def main(args=None):
    rclpy.init(args=args)
    node = ForceTorquePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
