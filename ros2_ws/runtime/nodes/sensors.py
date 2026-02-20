"""
Sensor nodes for Atlas spine runtime (lite mode).
IMU, Joint State, Force-Torque publishers.
"""
import math
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import atlas_ros2_lite as rclpy
from atlas_ros2_lite import (
    Node, make_imu, make_joint_state, make_wrench_stamped, make_header,
)

# 30 DOF joint names
JOINT_NAMES = [
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_pitch", "r_ankle_roll",
    "torso_yaw", "torso_pitch",
    "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
    "l_wrist_yaw", "l_wrist_roll", "l_wrist_pitch",
    "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
    "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch",
    "head_yaw", "head_pitch",
]

STANDING_POSE = {
    "l_hip_pitch": -0.1, "l_knee": 0.2, "l_ankle_pitch": -0.1,
    "r_hip_pitch": -0.1, "r_knee": 0.2, "r_ankle_pitch": -0.1,
}


class ImuPublisher(Node):
    def __init__(self, rate_hz=50):
        super().__init__("imu_publisher")
        self.pub = self.create_publisher(None, "/atlas/imu/data", 10)
        self.t0 = time.time()
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"IMU publisher started at {rate_hz}Hz (SIM)")

    def _tick(self):
        t = time.time() - self.t0
        msg = make_imu(
            header=make_header("imu_link"),
            orientation={"x": 0.0, "y": math.sin(t * 0.5) * 0.01, "z": 0.0, "w": 1.0},
            angular_velocity={"x": 0.0, "y": math.cos(t * 0.5) * 0.005, "z": 0.0},
            linear_acceleration={"x": 0.0, "y": 0.0, "z": 9.81},
        )
        self.pub.publish(msg)


class JointStatePublisher(Node):
    def __init__(self, rate_hz=50):
        super().__init__("joint_state_publisher")
        self.pub = self.create_publisher(None, "/atlas/joint_states", 10)
        self.t0 = time.time()
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"Joint state publisher started: {len(JOINT_NAMES)} joints at {rate_hz}Hz")

    def _tick(self):
        t = time.time() - self.t0
        positions = []
        for name in JOINT_NAMES:
            base = STANDING_POSE.get(name, 0.0)
            if name == "torso_pitch":
                base += math.sin(t * 0.8) * 0.005
            positions.append(base)
        msg = make_joint_state(
            header=make_header("base_link"),
            name=list(JOINT_NAMES),
            position=positions,
            velocity=[0.0] * len(JOINT_NAMES),
            effort=[0.0] * len(JOINT_NAMES),
        )
        self.pub.publish(msg)


class ForceTorquePublisher(Node):
    def __init__(self, rate_hz=50):
        super().__init__("force_torque_publisher")
        self.pub_left = self.create_publisher(None, "/atlas/ft/left_foot", 10)
        self.pub_right = self.create_publisher(None, "/atlas/ft/right_foot", 10)
        self.t0 = time.time()
        self.robot_mass_kg = 80.0
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"Force-torque publisher started at {rate_hz}Hz")

    def _tick(self):
        t = time.time() - self.t0
        total_weight = self.robot_mass_kg * 9.81
        sway = math.sin(t * 0.3) * 0.05
        fz_left = total_weight * (0.5 + sway)
        fz_right = total_weight * (0.5 - sway)
        self.pub_left.publish(make_wrench_stamped(
            header=make_header("l_foot_ft_link"),
            force={"x": 0.0, "y": 0.0, "z": fz_left},
        ))
        self.pub_right.publish(make_wrench_stamped(
            header=make_header("r_foot_ft_link"),
            force={"x": 0.0, "y": 0.0, "z": fz_right},
        ))
