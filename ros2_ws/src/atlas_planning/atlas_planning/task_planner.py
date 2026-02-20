"""
Task Planner Node
=================
High-level task decomposition using AI reasoning.
Receives natural language goals, decomposes into executable steps,
and publishes them for the supervisor/executor.

Bridges to: modules/humanoid/orchestrator/, atlas_adapter /agent/goal
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class TaskPlanner(Node):
    def __init__(self):
        super().__init__("task_planner")

        self.declare_parameter("atlas.supervisor.push_api_url", "http://127.0.0.1:8791")

        self.push_url = self.get_parameter("atlas.supervisor.push_api_url").value

        # Subscribe to task goals
        self.create_subscription(String, "/atlas/task/goal", self._goal_cb, 10)

        # Publish plans
        self.plan_pub = self.create_publisher(String, "/atlas/task/plan", 10)
        self.status_pub = self.create_publisher(String, "/atlas/task/status", 10)

        self.get_logger().info(f"Task planner started (API: {self.push_url})")

    def _goal_cb(self, msg: String):
        """Receive a goal and decompose it into a plan."""
        try:
            goal_data = json.loads(msg.data)
        except json.JSONDecodeError:
            goal_data = {"goal": msg.data, "mode": "plan_only", "depth": 1}

        goal = goal_data.get("goal", "")
        mode = goal_data.get("mode", "plan_only")
        depth = goal_data.get("depth", 1)

        self.get_logger().info(f"Planning goal: '{goal[:80]}' mode={mode} depth={depth}")

        # Publish status
        self._publish_status("planning", goal)

        # Try to call existing Atlas orchestrator via HTTP
        plan = self._call_orchestrator(goal, mode, depth)

        if plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            self._publish_status("planned", goal)
        else:
            # Fallback: simple decomposition
            fallback_plan = self._simple_decompose(goal)
            plan_msg = String()
            plan_msg.data = json.dumps(fallback_plan)
            self.plan_pub.publish(plan_msg)
            self._publish_status("planned_fallback", goal)

    def _call_orchestrator(self, goal: str, mode: str, depth: int) -> dict:
        """Call existing Atlas /agent/goal endpoint for AI-powered planning."""
        try:
            import urllib.request
            import urllib.error

            url = f"{self.push_url}/agent/goal"
            body = json.dumps({
                "goal": goal,
                "mode": mode,
                "depth": depth,
                "fast": True,
            }).encode("utf-8")

            req = urllib.request.Request(
                url, data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                if data.get("ok"):
                    return {
                        "goal": goal,
                        "mode": mode,
                        "source": "orchestrator",
                        "plan": data.get("data", {}).get("plan"),
                        "steps": data.get("data", {}).get("steps", []),
                        "output": data.get("data", {}).get("output"),
                        "model_used": data.get("data", {}).get("model_used"),
                    }
        except Exception as e:
            self.get_logger().warn(f"Orchestrator call failed: {e}")
        return None

    def _simple_decompose(self, goal: str) -> dict:
        """Fallback: rule-based task decomposition."""
        goal_lower = goal.lower()
        steps = []

        if any(w in goal_lower for w in ("camina", "walk", "ve a", "go to", "navega")):
            steps = [
                {"action": "localize", "description": "Determine current position via SLAM"},
                {"action": "plan_path", "description": "Compute path to target"},
                {"action": "execute_gait", "description": "Walk along planned path"},
                {"action": "verify", "description": "Confirm arrival at target"},
            ]
        elif any(w in goal_lower for w in ("agarra", "pick", "toma", "grab", "levanta")):
            steps = [
                {"action": "detect_object", "description": "Locate target object with vision"},
                {"action": "approach", "description": "Navigate to grasping distance"},
                {"action": "grasp", "description": "Execute grasp with manipulator"},
                {"action": "verify", "description": "Confirm object is held"},
            ]
        elif any(w in goal_lower for w in ("diagnostico", "health", "estado", "check")):
            steps = [
                {"action": "self_check", "description": "Run system diagnostics"},
                {"action": "report", "description": "Compile and publish health report"},
            ]
        else:
            steps = [
                {"action": "analyze", "description": f"Analyze goal: {goal[:100]}"},
                {"action": "plan", "description": "Generate execution plan"},
                {"action": "execute", "description": "Execute planned steps"},
                {"action": "verify", "description": "Verify completion"},
            ]

        return {
            "goal": goal,
            "mode": "plan_only",
            "source": "rule_based_fallback",
            "steps": steps,
        }

    def _publish_status(self, status: str, goal: str):
        msg = String()
        msg.data = json.dumps({"status": status, "goal": goal[:200], "ts": time.time()})
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
