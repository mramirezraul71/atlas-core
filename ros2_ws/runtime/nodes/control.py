"""
Control nodes for Atlas spine runtime (lite mode).
Balance controller, Gait generator, Joint commander.
"""
import math
import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import atlas_ros2_lite as rclpy
from atlas_ros2_lite import (
    Node, make_joint_state, make_vector3_stamped, make_string, make_header,
)


class BalanceController(Node):
    def __init__(self, rate_hz=100):
        super().__init__("balance_controller")
        self.kp, self.ki, self.kd = 50.0, 0.5, 5.0
        self._imu = None
        self._ft_left = None
        self._ft_right = None
        self._pitch_integral = 0.0
        self._prev_pitch_error = 0.0

        self.create_subscription(None, "/atlas/imu/data", self._imu_cb, 10)
        self.create_subscription(None, "/atlas/ft/left_foot", self._ft_left_cb, 10)
        self.create_subscription(None, "/atlas/ft/right_foot", self._ft_right_cb, 10)

        self.cmd_pub = self.create_publisher(None, "/atlas/joint_commands", 10)
        self.cop_pub = self.create_publisher(None, "/atlas/balance/cop", 10)

        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(f"Balance controller started: PID=({self.kp},{self.ki},{self.kd}) {rate_hz}Hz")

    def _imu_cb(self, msg): self._imu = msg
    def _ft_left_cb(self, msg): self._ft_left = msg
    def _ft_right_cb(self, msg): self._ft_right = msg

    def _loop(self):
        if not self._imu:
            return
        q = self._imu.get("orientation", {})
        pitch = 2.0 * (q.get("w", 1.0) * q.get("y", 0.0) - q.get("z", 0.0) * q.get("x", 0.0))

        # CoP
        cop_x = 0.0
        if self._ft_left and self._ft_right:
            fz_l = self._ft_left.get("wrench", {}).get("force", {}).get("z", 0)
            fz_r = self._ft_right.get("wrench", {}).get("force", {}).get("z", 0)
            total = fz_l + fz_r
            if total > 1.0:
                cop_x = (-0.1 * fz_l + 0.1 * fz_r) / total
        self.cop_pub.publish(make_vector3_stamped(
            header=make_header("base_link"),
            vector={"x": cop_x, "y": 0.0, "z": 0.0},
        ))

        # PID
        dt = 0.01
        error = -pitch
        self._pitch_integral = max(-1.0, min(1.0, self._pitch_integral + error * dt))
        d_error = (error - self._prev_pitch_error) / dt
        self._prev_pitch_error = error
        correction = self.kp * error + self.ki * self._pitch_integral + self.kd * d_error

        self.cmd_pub.publish(make_joint_state(
            header=make_header("base_link"),
            name=["l_ankle_pitch", "r_ankle_pitch"],
            effort=[correction, correction],
        ))


class GaitGenerator(Node):
    def __init__(self, rate_hz=100):
        super().__init__("gait_generator")
        self.gait_type = "stand"
        self._phase = "stand"
        self._phase_time = 0.0
        self._step_length = 0.15
        self._step_height = 0.04
        self._step_duration = 0.6

        self.create_subscription(None, "/atlas/gait/command", self._cmd_cb, 10)
        self.traj_pub = self.create_publisher(None, "/atlas/gait/trajectory", 10)
        self.phase_pub = self.create_publisher(None, "/atlas/gait/phase", 10)

        self.create_timer(1.0 / rate_hz, self._generate)
        self.get_logger().info(f"Gait generator started: type={self.gait_type}")

    def _cmd_cb(self, msg):
        cmd = (msg.get("data", "") if isinstance(msg, dict) else str(msg)).strip().lower()
        if cmd in ("stand", "walk", "trot", "stop"):
            self.gait_type = cmd if cmd != "stop" else "stand"
            self._phase = "stand"
            self._phase_time = 0.0
            self.get_logger().info(f"Gait command: {cmd}")

    def _generate(self):
        self.phase_pub.publish(make_string(self._phase))
        if self.gait_type == "stand":
            self._gen_stand()
        elif self.gait_type in ("walk", "trot"):
            self._gen_walk()

    def _gen_stand(self):
        self.traj_pub.publish(make_joint_state(
            name=["l_hip_pitch", "l_knee", "l_ankle_pitch", "r_hip_pitch", "r_knee", "r_ankle_pitch"],
            position=[-0.1, 0.2, -0.1, -0.1, 0.2, -0.1],
        ))

    def _gen_walk(self):
        dt = 0.01
        self._phase_time += dt
        t_norm = self._phase_time / self._step_duration
        if t_norm >= 1.0:
            self._phase_time = 0.0
            t_norm = 0.0
            self._phase = "left_swing" if self._phase != "left_swing" else "right_swing"

        sx = self._step_length * t_norm
        sz = self._step_height * math.sin(math.pi * t_norm)
        h_sw = -0.1 + math.atan2(sx, 0.5)
        k_sw = 0.2 + sz * 2.0
        a_sw = -0.1 - math.atan2(sx, 0.5) * 0.5

        if self._phase == "left_swing":
            pos = [h_sw, k_sw, a_sw, -0.1, 0.2, -0.1]
        elif self._phase == "right_swing":
            pos = [-0.1, 0.2, -0.1, h_sw, k_sw, a_sw]
        else:
            pos = [-0.1, 0.2, -0.1, -0.1, 0.2, -0.1]

        self.traj_pub.publish(make_joint_state(
            name=["l_hip_pitch", "l_knee", "l_ankle_pitch", "r_hip_pitch", "r_knee", "r_ankle_pitch"],
            position=pos,
        ))


class JointCommander(Node):
    def __init__(self, rate_hz=100):
        super().__init__("joint_commander")
        self._balance_cmd = None
        self._balance_ts = 0.0
        self._gait_cmd = None
        self._gait_ts = 0.0

        self.create_subscription(None, "/atlas/joint_commands", self._bal_cb, 10)
        self.create_subscription(None, "/atlas/gait/trajectory", self._gait_cb, 10)
        self.hw_pub = self.create_publisher(None, "/atlas/hw/joint_target", 10)

        self.create_timer(1.0 / rate_hz, self._dispatch)
        self.get_logger().info("Joint commander started (SIM)")

    def _bal_cb(self, msg):
        self._balance_cmd = msg
        self._balance_ts = time.time()

    def _gait_cb(self, msg):
        self._gait_cmd = msg
        self._gait_ts = time.time()

    def _dispatch(self):
        now = time.time()
        positions, efforts = {}, {}
        if self._gait_cmd and (now - self._gait_ts) < 0.1:
            for i, n in enumerate(self._gait_cmd.get("name", [])):
                p = self._gait_cmd.get("position", [])
                if i < len(p):
                    positions[n] = p[i]
        if self._balance_cmd and (now - self._balance_ts) < 0.1:
            for i, n in enumerate(self._balance_cmd.get("name", [])):
                e = self._balance_cmd.get("effort", [])
                if e and i < len(e):
                    efforts[n] = e[i]
        if not positions and not efforts:
            return
        joints = sorted(set(list(positions.keys()) + list(efforts.keys())))
        self.hw_pub.publish(make_joint_state(
            name=joints,
            position=[positions.get(j, 0.0) for j in joints],
            effort=[efforts.get(j, 0.0) for j in joints],
            velocity=[0.0] * len(joints),
        ))
