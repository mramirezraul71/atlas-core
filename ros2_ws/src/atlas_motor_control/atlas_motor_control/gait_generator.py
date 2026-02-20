"""
Gait Generator Node
===================
Generates walking trajectories for Atlas humanoid legs.
Supports: stand, walk, trot, custom gaits.

Architecture references:
- IHMC Open Robotics Software (step planning, swing trajectory)
- RoboParty / Roboto Origin full-stack bipedal locomotion
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time


class GaitPhase:
    STAND = "stand"
    LEFT_SWING = "left_swing"
    RIGHT_SWING = "right_swing"
    DOUBLE_SUPPORT = "double_support"


class GaitGenerator(Node):
    def __init__(self):
        super().__init__("gait_generator")

        self.declare_parameter("atlas.control.gait_type", "stand")
        self.declare_parameter("atlas.control.loop_rate_hz", 200)
        self.declare_parameter("atlas.planning.max_velocity", 0.5)

        self.gait_type = self.get_parameter("atlas.control.gait_type").value
        self.max_vel = self.get_parameter("atlas.planning.max_velocity").value
        rate = self.get_parameter("atlas.control.loop_rate_hz").value

        self._phase = GaitPhase.STAND
        self._phase_time = 0.0
        self._step_length = 0.15   # meters
        self._step_height = 0.04   # meters
        self._step_duration = 0.6  # seconds per step

        self.create_subscription(String, "/atlas/gait/command", self._gait_cmd_cb, 10)
        self.traj_pub = self.create_publisher(JointState, "/atlas/gait/trajectory", 10)
        self.phase_pub = self.create_publisher(String, "/atlas/gait/phase", 10)

        self.timer = self.create_timer(1.0 / rate, self._generate)
        self.get_logger().info(f"Gait generator started: type={self.gait_type}")

    def _gait_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ("stand", "walk", "trot", "stop"):
            self.gait_type = cmd if cmd != "stop" else "stand"
            self._phase = GaitPhase.STAND
            self._phase_time = 0.0
            self.get_logger().info(f"Gait command: {cmd}")

    def _generate(self):
        dt = 1.0 / 200.0
        phase_msg = String()
        phase_msg.data = self._phase
        self.phase_pub.publish(phase_msg)

        if self.gait_type == "stand":
            self._generate_stand()
        elif self.gait_type in ("walk", "trot"):
            self._generate_walk(dt)

    def _generate_stand(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "l_hip_pitch", "l_knee", "l_ankle_pitch",
            "r_hip_pitch", "r_knee", "r_ankle_pitch",
        ]
        msg.position = [-0.1, 0.2, -0.1, -0.1, 0.2, -0.1]
        self.traj_pub.publish(msg)

    def _generate_walk(self, dt: float):
        self._phase_time += dt
        t_norm = self._phase_time / self._step_duration

        if t_norm >= 1.0:
            self._phase_time = 0.0
            t_norm = 0.0
            if self._phase in (GaitPhase.STAND, GaitPhase.RIGHT_SWING):
                self._phase = GaitPhase.LEFT_SWING
            else:
                self._phase = GaitPhase.RIGHT_SWING

        swing_x = self._step_length * t_norm
        swing_z = self._step_height * math.sin(math.pi * t_norm)

        hip_swing = -0.1 + math.atan2(swing_x, 0.5)
        knee_swing = 0.2 + swing_z * 2.0
        ankle_swing = -0.1 - math.atan2(swing_x, 0.5) * 0.5

        hip_stance = -0.1
        knee_stance = 0.2
        ankle_stance = -0.1

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "l_hip_pitch", "l_knee", "l_ankle_pitch",
            "r_hip_pitch", "r_knee", "r_ankle_pitch",
        ]

        if self._phase == GaitPhase.LEFT_SWING:
            msg.position = [hip_swing, knee_swing, ankle_swing,
                            hip_stance, knee_stance, ankle_stance]
        elif self._phase == GaitPhase.RIGHT_SWING:
            msg.position = [hip_stance, knee_stance, ankle_stance,
                            hip_swing, knee_swing, ankle_swing]
        else:
            msg.position = [hip_stance, knee_stance, ankle_stance,
                            hip_stance, knee_stance, ankle_stance]

        self.traj_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
