"""
Balance Controller Node (IHMC-inspired)
=======================================
Subscribes to IMU and force-torque data to compute balance corrections.
Publishes joint torque/position commands to maintain upright posture.

Architecture reference: IHMC Open Robotics Software
- Whole-body control with momentum-based balance
- Center of Pressure (CoP) tracking
- Zero Moment Point (ZMP) regulation
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
import math


class BalanceController(Node):
    def __init__(self):
        super().__init__("balance_controller")

        self.declare_parameter("atlas.control.loop_rate_hz", 200)
        self.declare_parameter("atlas.control.balance_mode", "static")
        self.declare_parameter("atlas.control.pid_gains.kp", 50.0)
        self.declare_parameter("atlas.control.pid_gains.ki", 0.5)
        self.declare_parameter("atlas.control.pid_gains.kd", 5.0)

        self.balance_mode = self.get_parameter("atlas.control.balance_mode").value
        self.kp = self.get_parameter("atlas.control.pid_gains.kp").value
        self.ki = self.get_parameter("atlas.control.pid_gains.ki").value
        self.kd = self.get_parameter("atlas.control.pid_gains.kd").value

        # State
        self._imu_data = None
        self._ft_left = None
        self._ft_right = None
        self._joint_state = None
        self._pitch_error_integral = 0.0
        self._prev_pitch_error = 0.0

        # Subscribers
        self.create_subscription(Imu, "/atlas/imu/data", self._imu_cb, 10)
        self.create_subscription(WrenchStamped, "/atlas/ft/left_foot", self._ft_left_cb, 10)
        self.create_subscription(WrenchStamped, "/atlas/ft/right_foot", self._ft_right_cb, 10)
        self.create_subscription(JointState, "/atlas/joint_states", self._joint_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(JointState, "/atlas/joint_commands", 10)
        self.cop_pub = self.create_publisher(Vector3Stamped, "/atlas/balance/cop", 10)

        rate = self.get_parameter("atlas.control.loop_rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f"Balance controller started: mode={self.balance_mode}, "
            f"PID=({self.kp}, {self.ki}, {self.kd}), rate={rate}Hz"
        )

    def _imu_cb(self, msg: Imu):
        self._imu_data = msg

    def _ft_left_cb(self, msg: WrenchStamped):
        self._ft_left = msg

    def _ft_right_cb(self, msg: WrenchStamped):
        self._ft_right = msg

    def _joint_cb(self, msg: JointState):
        self._joint_state = msg

    def _compute_cop(self) -> tuple:
        """Compute Center of Pressure from foot force-torque sensors."""
        if not self._ft_left or not self._ft_right:
            return 0.0, 0.0
        fz_l = self._ft_left.wrench.force.z
        fz_r = self._ft_right.wrench.force.z
        total = fz_l + fz_r
        if total < 1.0:
            return 0.0, 0.0
        foot_spacing = 0.1  # meters from center
        cop_x = (-foot_spacing * fz_l + foot_spacing * fz_r) / total
        cop_y = 0.0
        return cop_x, cop_y

    def _extract_pitch(self) -> float:
        """Extract pitch angle from IMU quaternion (simplified)."""
        if not self._imu_data:
            return 0.0
        q = self._imu_data.orientation
        pitch = 2.0 * (q.w * q.y - q.z * q.x)
        return pitch

    def _control_loop(self):
        if self._imu_data is None:
            return

        pitch = self._extract_pitch()
        cop_x, cop_y = self._compute_cop()

        # Publish CoP for monitoring
        cop_msg = Vector3Stamped()
        cop_msg.header.stamp = self.get_clock().now().to_msg()
        cop_msg.header.frame_id = "base_link"
        cop_msg.vector.x = cop_x
        cop_msg.vector.y = cop_y
        cop_msg.vector.z = 0.0
        self.cop_pub.publish(cop_msg)

        if self.balance_mode == "static":
            self._static_balance(pitch)
        elif self.balance_mode == "dynamic":
            self._dynamic_balance(pitch, cop_x)
        elif self.balance_mode == "impulse":
            self._impulse_balance(pitch, cop_x)

    def _static_balance(self, pitch: float):
        """Simple PID on pitch to keep upright (standing still)."""
        dt = 1.0 / 200.0
        error = 0.0 - pitch
        self._pitch_error_integral += error * dt
        self._pitch_error_integral = max(-1.0, min(1.0, self._pitch_error_integral))
        d_error = (error - self._prev_pitch_error) / dt
        self._prev_pitch_error = error

        correction = self.kp * error + self.ki * self._pitch_error_integral + self.kd * d_error

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ["l_ankle_pitch", "r_ankle_pitch"]
        cmd.effort = [correction, correction]
        self.cmd_pub.publish(cmd)

    def _dynamic_balance(self, pitch: float, cop_x: float):
        """ZMP-based balance for walking (placeholder for IHMC-style control)."""
        # TODO: Implement full ZMP tracking with preview control
        self._static_balance(pitch)

    def _impulse_balance(self, pitch: float, cop_x: float):
        """Momentum-based impulse control (IHMC whole-body control reference)."""
        # TODO: Implement centroidal momentum control
        self._static_balance(pitch)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
