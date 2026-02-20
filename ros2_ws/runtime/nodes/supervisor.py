"""
Supervisor and Brain Bridge nodes for Atlas spine runtime (lite mode).
Bridges to existing PUSH (8791) and ROBOT (8002) backends.
"""
import json
import time
import threading
import urllib.request
import urllib.error
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import atlas_ros2_lite as rclpy
from atlas_ros2_lite import Node, make_string


class SupervisorNode(Node):
    def __init__(self, push_url="http://127.0.0.1:8791", robot_url="http://127.0.0.1:8002"):
        super().__init__("atlas_supervisor")
        self.push_url = push_url
        self.robot_url = robot_url
        self.governance_mode = "governed"
        self._push_ok = False
        self._robot_ok = False
        self._health_score = 0.0
        self._subsystems = {}

        self.create_subscription(None, "/atlas/task/status", self._task_cb, 10)
        self.create_subscription(None, "/atlas/perception/detections", self._perc_cb, 10)
        self.create_subscription(None, "/atlas/gait/phase", self._gait_cb, 10)
        self.create_subscription(None, "/atlas/slam/active", self._slam_cb, 10)

        self.hb_pub = self.create_publisher(None, "/atlas/supervisor/heartbeat", 10)
        self.gov_pub = self.create_publisher(None, "/atlas/supervisor/governance", 10)
        self.alert_pub = self.create_publisher(None, "/atlas/supervisor/alert", 10)
        self.state_pub = self.create_publisher(None, "/atlas/supervisor/state", 10)

        self.create_timer(5.0, self._heartbeat)
        self.create_timer(30.0, self._health_check)
        self.create_timer(10.0, self._sync_governance)
        self.get_logger().info(f"Supervisor started: PUSH={push_url}, ROBOT={robot_url}")

    def _task_cb(self, msg): pass
    def _perc_cb(self, msg):
        try:
            d = json.loads(msg.get("data", "{}")) if isinstance(msg, dict) else {}
            if d.get("count", 0) > 0:
                self._subsystems["perception"] = {"active": True, "ts": time.time()}
        except Exception:
            pass
    def _gait_cb(self, msg):
        self._subsystems["gait"] = {"phase": msg.get("data", ""), "ts": time.time()}
    def _slam_cb(self, msg):
        self._subsystems["slam"] = {"active": msg.get("data", False), "ts": time.time()}

    def _check_url(self, url):
        try:
            with urllib.request.urlopen(urllib.request.Request(url), timeout=3) as r:
                return r.status == 200
        except Exception:
            return False

    def _heartbeat(self):
        self._push_ok = self._check_url(f"{self.push_url}/status")
        self._robot_ok = self._check_url(f"{self.robot_url}/status")
        hb = {
            "ts": time.time(), "governance": self.governance_mode,
            "push": self._push_ok, "robot": self._robot_ok,
            "health": self._health_score, "subsystems": list(self._subsystems.keys()),
        }
        self.hb_pub.publish(make_string(json.dumps(hb)))
        if not self._push_ok:
            self.alert_pub.publish(make_string(json.dumps({"level": "warn", "msg": "PUSH unreachable"})))

    def _health_check(self):
        score = 50.0
        if self._push_ok: score += 25.0
        if self._robot_ok: score += 15.0
        if self._subsystems.get("perception", {}).get("active"): score += 5.0
        if self._subsystems.get("slam", {}).get("active"): score += 5.0
        try:
            with urllib.request.urlopen(f"{self.push_url}/health", timeout=5) as r:
                d = json.loads(r.read().decode())
                score = (score + d.get("score", 0)) / 2.0
        except Exception:
            pass
        self._health_score = min(100.0, score)
        self.state_pub.publish(make_string(json.dumps({
            "health": round(self._health_score, 1), "push": self._push_ok,
            "robot": self._robot_ok, "governance": self.governance_mode,
        })))

    def _sync_governance(self):
        try:
            with urllib.request.urlopen(f"{self.push_url}/governance/status", timeout=3) as r:
                d = json.loads(r.read().decode())
                new = d.get("mode") or d.get("governance_mode")
                if new and new != self.governance_mode:
                    old = self.governance_mode
                    self.governance_mode = new
                    self.get_logger().info(f"Governance: {old} -> {new}")
                    self.gov_pub.publish(make_string(json.dumps({"mode": new, "prev": old})))
        except Exception:
            pass


class BrainBridgeNode(Node):
    def __init__(self, push_url="http://127.0.0.1:8791"):
        super().__init__("brain_bridge")
        self.push_url = push_url
        self.create_subscription(None, "/atlas/brain/query", self._query_cb, 10)
        self.resp_pub = self.create_publisher(None, "/atlas/brain/response", 10)
        self.status_pub = self.create_publisher(None, "/atlas/brain/status", 10)
        self.create_timer(15.0, self._sync_status)
        self.get_logger().info(f"Brain bridge started (API: {push_url})")

    def _query_cb(self, msg):
        data = msg if isinstance(msg, dict) else {"query": str(msg.get("data", msg))}
        query = data.get("query", "")
        model = data.get("model", "auto")
        qid = data.get("query_id", f"ros2_{int(time.time()*1000)}")
        self.get_logger().info(f"Brain query [{qid}]: '{query[:50]}'")
        threading.Thread(target=self._process, args=(qid, query, model), daemon=True).start()

    def _process(self, qid, query, model):
        t0 = time.time()
        try:
            body = json.dumps({"goal": query, "mode": "plan_only", "depth": 1, "fast": True}).encode()
            req = urllib.request.Request(
                f"{self.push_url}/agent/goal", data=body,
                headers={"Content-Type": "application/json"}, method="POST",
            )
            with urllib.request.urlopen(req, timeout=60) as r:
                result = json.loads(r.read().decode())
            ms = round((time.time() - t0) * 1000, 1)
            if result.get("ok"):
                self.resp_pub.publish(make_string(json.dumps({
                    "query_id": qid, "success": True, "ms": ms,
                    "response": (result.get("data") or {}).get("output", ""),
                    "model_used": (result.get("data") or {}).get("model_used", ""),
                })))
                return
            self.resp_pub.publish(make_string(json.dumps({
                "query_id": qid, "success": False, "ms": ms,
                "error": result.get("error", "Unknown"),
            })))
        except Exception as e:
            self.resp_pub.publish(make_string(json.dumps({
                "query_id": qid, "success": False,
                "error": str(e), "ms": round((time.time() - t0) * 1000, 1),
            })))

    def _sync_status(self):
        status = {"ts": time.time(), "subsystems": {}}
        for name, ep in [("brain", "/brain/status"), ("cognitive", "/cognitive/status"),
                         ("nervous", "/nervous/status"), ("governance", "/governance/status")]:
            try:
                with urllib.request.urlopen(f"{self.push_url}{ep}", timeout=3) as r:
                    status["subsystems"][name] = {"ok": True}
            except Exception:
                status["subsystems"][name] = {"ok": False}
        self.status_pub.publish(make_string(json.dumps(status)))
