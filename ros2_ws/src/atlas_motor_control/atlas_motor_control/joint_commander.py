"""
Joint Commander Node
====================
Receives joint commands from balance controller and gait generator,
merges them with priority, and dispatches to hardware (or simulation).

Priority: balance corrections > gait trajectory > default pose
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time


class JointCommander(Node):
    def __init__(self):
        super().__init__("joint_commander")

        self.declare_parameter("atlas.control.loop_rate_hz", 200)
        self.declare_parameter("atlas.hardware.enable_hardware", False)
        self.declare_parameter("atlas.hardware.joint_count", 30)

        rate = self.get_parameter("atlas.control.loop_rate_hz").value
        self.hw_enabled = self.get_parameter("atlas.hardware.enable_hardware").value
        self.joint_count = self.get_parameter("atlas.hardware.joint_count").value

        # Latest commands from each source
        self._balance_cmd = None
        self._balance_ts = 0.0
        self._gait_cmd = None
        self._gait_ts = 0.0
        self._cmd_timeout = 0.1  # seconds before a command source is considered stale

        # Subscribers
        self.create_subscription(JointState, "/atlas/joint_commands", self._balance_cb, 10)
        self.create_subscription(JointState, "/atlas/gait/trajectory", self._gait_cb, 10)

        # Publisher: final merged commands to hardware/sim
        self.hw_pub = self.create_publisher(JointState, "/atlas/hw/joint_target", 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, "/atlas/hw/debug_efforts", 10)

        self.timer = self.create_timer(1.0 / rate, self._dispatch)
        self.get_logger().info(
            f"Joint commander started: hw={'ON' if self.hw_enabled else 'SIM'}, "
            f"joints={self.joint_count}, rate={rate}Hz"
        )

    def _balance_cb(self, msg: JointState):
        self._balance_cmd = msg
        self._balance_ts = time.time()

    def _gait_cb(self, msg: JointState):
        self._gait_cmd = msg
        self._gait_ts = time.time()

    def _dispatch(self):
        now = time.time()
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        # Start with gait trajectory as base
        positions = {}
        efforts = {}

        if self._gait_cmd and (now - self._gait_ts) < self._cmd_timeout:
            for i, name in enumerate(self._gait_cmd.name):
                if i < len(self._gait_cmd.position):
                    positions[name] = self._gait_cmd.position[i]

        # Override with balance corrections (higher priority)
        if self._balance_cmd and (now - self._balance_ts) < self._cmd_timeout:
            for i, name in enumerate(self._balance_cmd.name):
                if self._balance_cmd.effort and i < len(self._balance_cmd.effort):
                    efforts[name] = self._balance_cmd.effort[i]
                if self._balance_cmd.position and i < len(self._balance_cmd.position):
                    positions[name] = self._balance_cmd.position[i]

        if not positions and not efforts:
            return

        all_joints = sorted(set(list(positions.keys()) + list(efforts.keys())))
        merged.name = all_joints
        merged.position = [positions.get(j, 0.0) for j in all_joints]
        merged.effort = [efforts.get(j, 0.0) for j in all_joints]
        merged.velocity = [0.0] * len(all_joints)

        self.hw_pub.publish(merged)

        if self.hw_enabled:
            # TODO: Send to real motor drivers via CAN/serial
            pass

        # Debug: publish effort array
        dbg = Float64MultiArray()
        dbg.data = merged.effort
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
