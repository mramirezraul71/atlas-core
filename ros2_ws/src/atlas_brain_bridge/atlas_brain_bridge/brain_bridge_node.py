"""
Brain Bridge Node
=================
Connects the ROS 2 spine to the existing Atlas AI systems:
  - AI Consultant (brain/learning/ai_consultant.py) via Bedrock/Direct Anthropic
  - ANS (modules/humanoid/ans/) — Autonomous Nervous System
  - Memory Engine (modules/humanoid/memory_engine/) — episodic, semantic, autobiographical
  - Orchestrator (modules/humanoid/orchestrator/) — goal planning and execution
  - Governance (modules/humanoid/governance/) — mode enforcement

This node acts as a ROS 2 service server that other nodes can call
to request AI reasoning, memory queries, or system diagnostics.

Bridges to: PUSH backend (8791) HTTP API
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import urllib.request
import urllib.error
import threading


class BrainBridgeNode(Node):
    def __init__(self):
        super().__init__("brain_bridge")

        self.declare_parameter("atlas.supervisor.push_api_url", "http://127.0.0.1:8791")
        self.declare_parameter("atlas.supervisor.ai_backend", "bedrock")

        self.push_url = self.get_parameter("atlas.supervisor.push_api_url").value
        self.ai_backend = self.get_parameter("atlas.supervisor.ai_backend").value

        # Subscribers
        self.create_subscription(String, "/atlas/brain/query", self._query_cb, 10)
        self.create_subscription(String, "/atlas/brain/memory_query", self._memory_cb, 10)

        # Publishers
        self.response_pub = self.create_publisher(String, "/atlas/brain/response", 10)
        self.memory_pub = self.create_publisher(String, "/atlas/brain/memory_result", 10)
        self.status_pub = self.create_publisher(String, "/atlas/brain/status", 10)

        # Periodic brain status sync
        self.create_timer(15.0, self._sync_brain_status)

        self.get_logger().info(
            f"Brain bridge started: API={self.push_url}, backend={self.ai_backend}"
        )

    def _query_cb(self, msg: String):
        """Handle AI reasoning query from any ROS 2 node."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"query": msg.data, "model": "auto"}

        query = data.get("query", "")
        model = data.get("model", "auto")
        query_id = data.get("query_id", f"ros2_{int(time.time()*1000)}")

        self.get_logger().info(f"Brain query [{query_id}]: '{query[:60]}' model={model}")

        # Run in thread to avoid blocking the ROS 2 executor
        thread = threading.Thread(
            target=self._process_query,
            args=(query_id, query, model),
            daemon=True,
        )
        thread.start()

    def _process_query(self, query_id: str, query: str, model: str):
        """Call PUSH backend /agent/goal for AI reasoning."""
        t0 = time.time()
        try:
            body = json.dumps({
                "goal": query,
                "mode": "plan_only",
                "depth": 1,
                "fast": True,
                "model": model if model != "auto" else None,
            }).encode("utf-8")

            req = urllib.request.Request(
                f"{self.push_url}/agent/goal",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=60) as resp:
                result = json.loads(resp.read().decode("utf-8"))

            latency = round((time.time() - t0) * 1000, 1)

            if result.get("ok"):
                output = (result.get("data") or {}).get("output", "")
                model_used = (result.get("data") or {}).get("model_used", "unknown")
                response = {
                    "query_id": query_id,
                    "success": True,
                    "response": output,
                    "model_used": model_used,
                    "latency_ms": latency,
                }
            else:
                response = {
                    "query_id": query_id,
                    "success": False,
                    "error": result.get("error", "Unknown error"),
                    "latency_ms": latency,
                }
        except Exception as e:
            response = {
                "query_id": query_id,
                "success": False,
                "error": str(e),
                "latency_ms": round((time.time() - t0) * 1000, 1),
            }

        msg = String()
        msg.data = json.dumps(response)
        self.response_pub.publish(msg)

    def _memory_cb(self, msg: String):
        """Handle memory queries (episodic, semantic, libro de vida)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"type": "search", "query": msg.data}

        mem_type = data.get("type", "search")
        query = data.get("query", "")

        thread = threading.Thread(
            target=self._process_memory_query,
            args=(mem_type, query),
            daemon=True,
        )
        thread.start()

    def _process_memory_query(self, mem_type: str, query: str):
        """Query Atlas memory systems via PUSH API."""
        endpoints = {
            "episodic": "/api/cognitive-memory/episodic/search",
            "semantic": "/api/cognitive-memory/semantic/search",
            "lifelog": "/api/cognitive-memory/lifelog/status",
            "libro_vida": "/api/cognitive-memory/libro-vida/search",
        }

        endpoint = endpoints.get(mem_type, endpoints["episodic"])

        try:
            if "search" in endpoint:
                body = json.dumps({"query": query, "limit": 5}).encode("utf-8")
                req = urllib.request.Request(
                    f"{self.push_url}{endpoint}",
                    data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
            else:
                req = urllib.request.Request(
                    f"{self.push_url}{endpoint}", method="GET"
                )

            with urllib.request.urlopen(req, timeout=10) as resp:
                result = json.loads(resp.read().decode("utf-8"))

            response = {
                "type": mem_type,
                "query": query,
                "success": True,
                "data": result,
            }
        except Exception as e:
            response = {
                "type": mem_type,
                "query": query,
                "success": False,
                "error": str(e),
            }

        msg = String()
        msg.data = json.dumps(response)
        self.memory_pub.publish(msg)

    def _sync_brain_status(self):
        """Periodically publish brain/ANS status."""
        status = {"ts": time.time(), "subsystems": {}}

        endpoints = {
            "brain": "/brain/status",
            "cognitive": "/cognitive/status",
            "nervous": "/nervous/status",
            "governance": "/governance/status",
        }

        for name, endpoint in endpoints.items():
            try:
                req = urllib.request.Request(
                    f"{self.push_url}{endpoint}", method="GET"
                )
                with urllib.request.urlopen(req, timeout=3) as resp:
                    data = json.loads(resp.read().decode("utf-8"))
                    status["subsystems"][name] = {"ok": True, "data": data}
            except Exception as e:
                status["subsystems"][name] = {"ok": False, "error": str(e)}

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrainBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
