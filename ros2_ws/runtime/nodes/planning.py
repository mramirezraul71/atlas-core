"""
Planning nodes for Atlas spine runtime (lite mode).
Task planner (bridges to PUSH /agent/goal) and Path planner.
"""
import json
import math
import time
import threading
import urllib.request
import urllib.error
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import atlas_ros2_lite as rclpy
from atlas_ros2_lite import Node, make_string, make_header


class TaskPlanner(Node):
    def __init__(self, push_url="http://127.0.0.1:8791"):
        super().__init__("task_planner")
        self.push_url = push_url
        self.create_subscription(None, "/atlas/task/goal", self._goal_cb, 10)
        self.plan_pub = self.create_publisher(None, "/atlas/task/plan", 10)
        self.status_pub = self.create_publisher(None, "/atlas/task/status", 10)
        self.get_logger().info(f"Task planner started (API: {self.push_url})")

    def _goal_cb(self, msg):
        data = msg if isinstance(msg, dict) else {"goal": str(msg.get("data", msg))}
        goal = data.get("goal", "")
        mode = data.get("mode", "plan_only")
        self.get_logger().info(f"Planning: '{goal[:60]}' mode={mode}")
        self.status_pub.publish(make_string(json.dumps({"status": "planning", "goal": goal[:200]})))
        t = threading.Thread(target=self._plan, args=(goal, mode), daemon=True)
        t.start()

    def _plan(self, goal, mode):
        try:
            body = json.dumps({"goal": goal, "mode": mode, "depth": 1, "fast": True}).encode()
            req = urllib.request.Request(
                f"{self.push_url}/agent/goal", data=body,
                headers={"Content-Type": "application/json"}, method="POST",
            )
            with urllib.request.urlopen(req, timeout=30) as resp:
                result = json.loads(resp.read().decode())
            if result.get("ok"):
                plan = {
                    "goal": goal, "source": "orchestrator",
                    "output": (result.get("data") or {}).get("output"),
                    "steps": (result.get("data") or {}).get("steps", []),
                    "model_used": (result.get("data") or {}).get("model_used"),
                }
                self.plan_pub.publish(make_string(json.dumps(plan)))
                self.status_pub.publish(make_string(json.dumps({"status": "planned", "goal": goal[:200]})))
                return
        except Exception as e:
            self.get_logger().warn(f"Orchestrator call failed: {e}")

        # Fallback
        self.plan_pub.publish(make_string(json.dumps({
            "goal": goal, "source": "fallback",
            "steps": [{"action": "analyze"}, {"action": "plan"}, {"action": "execute"}, {"action": "verify"}],
        })))
        self.status_pub.publish(make_string(json.dumps({"status": "planned_fallback", "goal": goal[:200]})))


class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        self._pose = None
        self.create_subscription(None, "/atlas/slam/pose", self._pose_cb, 10)
        self.create_subscription(None, "/atlas/nav/goal", self._goal_cb, 10)
        self.path_pub = self.create_publisher(None, "/atlas/nav/path", 10)
        self.status_pub = self.create_publisher(None, "/atlas/nav/status", 10)
        self.get_logger().info("Path planner started")

    def _pose_cb(self, msg):
        self._pose = msg

    def _goal_cb(self, msg):
        if not self._pose:
            self.get_logger().warn("No pose, cannot plan path")
            return
        gp = msg.get("pose", {}).get("position", {})
        sp = self._pose.get("pose", {}).get("position", {})
        gx, gy = gp.get("x", 0), gp.get("y", 0)
        sx, sy = sp.get("x", 0), sp.get("y", 0)
        dist = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)
        n = max(2, int(dist / 0.1))
        waypoints = []
        for i in range(n + 1):
            t = i / n
            waypoints.append({"x": sx + t * (gx - sx), "y": sy + t * (gy - sy)})
        self.path_pub.publish(make_string(json.dumps({"waypoints": waypoints, "count": len(waypoints)})))
        self.status_pub.publish(make_string(json.dumps({"status": "ready", "waypoints": len(waypoints)})))
