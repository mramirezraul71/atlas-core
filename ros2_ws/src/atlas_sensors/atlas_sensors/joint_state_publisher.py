"""
Joint State Publisher Node
==========================
Publishes joint positions, velocities, and efforts for all Atlas DOF.
In simulation mode, publishes a default standing pose.
In hardware mode, reads from motor encoders.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

# Atlas humanoid joint names (30 DOF)
JOINT_NAMES = [
    # Left leg (6)
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
    # Right leg (6)
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll",
    # Torso (2)
    "torso_yaw", "torso_pitch",
    # Left arm (7)
    "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
    "l_wrist_yaw", "l_wrist_roll", "l_wrist_pitch",
    # Right arm (7)
    "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
    "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch",
    # Head (2)
    "head_yaw", "head_pitch",
]

# Default standing pose (radians)
STANDING_POSE = {
    "l_hip_pitch": -0.1, "l_knee": 0.2, "l_ankle_pitch": -0.1,
    "r_hip_pitch": -0.1, "r_knee": 0.2, "r_ankle_pitch": -0.1,
}


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("joint_state_publisher")
        self.declare_parameter("atlas.hardware.sensor_rate_hz", 50)
        self.declare_parameter("atlas.hardware.enable_hardware", False)
        self.declare_parameter("atlas.hardware.joint_count", 30)

        rate = self.get_parameter("atlas.hardware.sensor_rate_hz").value
        self.hw_enabled = self.get_parameter("atlas.hardware.enable_hardware").value

        self.publisher_ = self.create_publisher(JointState, "/atlas/joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.t0 = time.time()
        self.get_logger().info(
            f"Joint state publisher started: {len(JOINT_NAMES)} joints at {rate}Hz"
        )

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)

        if self.hw_enabled:
            # TODO: read from real motor encoders
            msg.position = [0.0] * len(JOINT_NAMES)
        else:
            # Simulation: standing pose with subtle breathing motion
            t = time.time() - self.t0
            positions = []
            for name in JOINT_NAMES:
                base = STANDING_POSE.get(name, 0.0)
                # Add subtle breathing oscillation to torso
                if name == "torso_pitch":
                    base += math.sin(t * 0.8) * 0.005
                positions.append(base)
            msg.position = positions

        msg.velocity = [0.0] * len(JOINT_NAMES)
        msg.effort = [0.0] * len(JOINT_NAMES)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
