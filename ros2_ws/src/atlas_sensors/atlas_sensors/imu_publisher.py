"""
IMU Publisher Node
==================
Publishes IMU data (orientation, angular velocity, linear acceleration).
In simulation mode, generates synthetic data.
In hardware mode, reads from real IMU over serial/CAN.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import time


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.declare_parameter("atlas.hardware.sensor_rate_hz", 50)
        self.declare_parameter("atlas.hardware.enable_hardware", False)

        rate = self.get_parameter("atlas.hardware.sensor_rate_hz").value
        self.hw_enabled = self.get_parameter("atlas.hardware.enable_hardware").value

        self.publisher_ = self.create_publisher(Imu, "/atlas/imu/data", 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.t0 = time.time()
        self.get_logger().info(
            f"IMU publisher started at {rate}Hz (hw={'ON' if self.hw_enabled else 'SIM'})"
        )

    def timer_callback(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        if self.hw_enabled:
            # TODO: read from real IMU hardware (serial/CAN/I2C)
            pass
        else:
            # Simulation: gentle oscillation to simulate standing balance
            t = time.time() - self.t0
            msg.orientation.x = 0.0
            msg.orientation.y = math.sin(t * 0.5) * 0.01  # small pitch sway
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = math.cos(t * 0.5) * 0.005
            msg.angular_velocity.z = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = 9.81

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
