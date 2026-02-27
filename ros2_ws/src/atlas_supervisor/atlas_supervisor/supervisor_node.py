"""
Supervisor Node (Supervisor.AtlasMaster)
========================================
Top-level orchestrator for the Atlas ROS 2 spine.
Responsibilities:
  - Heartbeat to all subsystems
  - Bridge to existing PUSH (8791) and ROBOT (8002) backends
  - Governance enforcement (governed/growth/emergency modes)
  - Task dispatch and monitoring
  - System health aggregation

Bridges to: atlas_adapter (PUSH), nexus (ROBOT), modules/humanoid/governance/
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import json
import time
import urllib.request
import urllib.error


class SupervisorNode(Node):
    def __init__(self):
        super().__init__("atlas_supervisor")

        self.declare_parameter("atlas.supervisor.push_api_url", "http://127.0.0.1:8791")
        self.declare_parameter("atlas.supervisor.robot_api_url", "http://127.0.0.1:8002")
        self.declare_parameter("atlas.supervisor.heartbeat_interval_sec", 5)
        self.declare_parameter("atlas.supervisor.governance_mode", "governed")
        self.declare_parameter("atlas.supervisor.ai_backend", "bedrock")

        self.push_url = self.get_parameter("atlas.supervisor.push_api_url").value
        self.robot_url = self.get_parameter("atlas.supervisor.robot_api_url").value
        self.hb_interval = self.get_parameter("atlas.supervisor.heartbeat_interval_sec").value
        self.governance_mode = self.get_parameter("atlas.supervisor.governance_mode").value
        self.ai_backend = self.get_parameter("atlas.supervisor.ai_backend").value

        # State tracking
        self._push_reachable = False
        self._robot_reachable = False
        self._active_tasks = []
        self._health_score = 0.0
        self._subsystem_status = {}

        # Subscribers (listen to all subsystems)
        self.create_subscription(String, "/atlas/task/status", self._task_status_cb, 10)
        self.create_subscription(String, "/atlas/perception/detections", self._perception_cb, 10)
        self.create_subscription(String, "/atlas/gait/phase", self._gait_phase_cb, 10)
        self.create_subscription(Bool, "/atlas/slam/active", self._slam_status_cb, 10)

        # Publishers
        self.heartbeat_pub = self.create_publisher(String, "/atlas/supervisor/heartbeat", 10)
        self.governance_pub = self.create_publisher(String, "/atlas/supervisor/governance", 10)
        self.alert_pub = self.create_publisher(String, "/atlas/supervisor/alert", 10)
        self.state_pub = self.create_publisher(String, "/atlas/supervisor/state", 10)

        # Timers
        self.create_timer(self.hb_interval, self._heartbeat_loop)
        self.create_timer(30.0, self._health_check_loop)
        self.create_timer(10.0, self._sync_governance)

        self.get_logger().info(
            f"Supervisor started: PUSH={self.push_url}, ROBOT={self.robot_url}, "
            f"governance={self.governance_mode}, ai={self.ai_backend}"
        )

    # ── Callbacks ──

    def _task_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            status = data.get("status", "")
            goal = data.get("goal", "")[:80]
            self.get_logger().info(f"Task status: {status} — {goal}")
        except Exception:
            pass

    def _perception_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            count = data.get("count", 0)
            if count > 0:
                self._subsystem_status["perception"] = {
                    "active": True, "detections": count, "ts": time.time()
                }
        except Exception:
            pass

    def _gait_phase_cb(self, msg: String):
        self._subsystem_status["gait"] = {"phase": msg.data, "ts": time.time()}

    def _slam_status_cb(self, msg: Bool):
        self._subsystem_status["slam"] = {"active": msg.data, "ts": time.time()}

    # ── Heartbeat ──

    def _heartbeat_loop(self):
        """Periodic heartbeat: check backends and publish system state."""
        self._push_reachable = self._check_endpoint(f"{self.push_url}/status")
        self._robot_reachable = self._check_endpoint(f"{self.robot_url}/status")

        heartbeat = {
            "ts": time.time(),
            "governance_mode": self.governance_mode,
            "push_reachable": self._push_reachable,
            "robot_reachable": self._robot_reachable,
            "active_tasks": len(self._active_tasks),
            "health_score": self._health_score,
            "subsystems": list(self._subsystem_status.keys()),
        }

        msg = String()
        msg.data = json.dumps(heartbeat)
        self.heartbeat_pub.publish(msg)

        # Alert if backend unreachable
        if not self._push_reachable:
            self._publish_alert("warn", "PUSH backend unreachable", self.push_url)
        if not self._robot_reachable:
            self._publish_alert("info", "ROBOT backend unreachable", self.robot_url)

    def _check_endpoint(self, url: str) -> bool:
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=3) as resp:
                return resp.status == 200
        except Exception:
            return False

    # ── Health Check ──

    def _health_check_loop(self):
        """Periodic comprehensive health check via PUSH backend."""
        score = 50.0  # base score
        if self._push_reachable:
            score += 25.0
        if self._robot_reachable:
            score += 15.0
        if self._subsystem_status.get("perception", {}).get("active"):
            score += 5.0
        if self._subsystem_status.get("slam", {}).get("active"):
            score += 5.0

        self._health_score = min(100.0, score)

        # Try to get detailed health from PUSH
        try:
            req = urllib.request.Request(f"{self.push_url}/health", method="GET")
            with urllib.request.urlopen(req, timeout=5) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                push_score = data.get("score", 0)
                self._health_score = (self._health_score + push_score) / 2.0
        except Exception:
            pass

        state_msg = String()
        state_msg.data = json.dumps({
            "health_score": round(self._health_score, 1),
            "push": self._push_reachable,
            "robot": self._robot_reachable,
            "governance": self.governance_mode,
            "subsystems": self._subsystem_status,
            "ts": time.time(),
        })
        self.state_pub.publish(state_msg)

    # ── Governance ──

    def _sync_governance(self):
        """Sync governance mode from PUSH backend."""
        try:
            req = urllib.request.Request(
                f"{self.push_url}/governance/status", method="GET"
            )
            with urllib.request.urlopen(req, timeout=3) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                new_mode = data.get("mode") or data.get("governance_mode")
                if new_mode and new_mode != self.governance_mode:
                    old = self.governance_mode
                    self.governance_mode = new_mode
                    self.get_logger().info(f"Governance mode changed: {old} -> {new_mode}")
                    gov_msg = String()
                    gov_msg.data = json.dumps({
                        "mode": new_mode, "previous": old, "ts": time.time()
                    })
                    self.governance_pub.publish(gov_msg)
        except Exception:
            pass

    # ── Alerts ──

    def _publish_alert(self, level: str, message: str, detail: str = ""):
        msg = String()
        msg.data = json.dumps({
            "level": level, "message": message, "detail": detail, "ts": time.time()
        })
        self.alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
