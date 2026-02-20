"""
Atlas Spine Launcher
====================
Starts all 5 layers of the Atlas ROS 2 spine using the lite runtime.
No ROS 2 installation required — pure Python pub/sub bus.

Usage:
  python launch_spine.py                  # full spine (simulation)
  python launch_spine.py --layers 1,2     # only hardware + control
  python launch_spine.py --monitor        # full spine + live topic monitor
  python launch_spine.py --test           # quick self-test (5 seconds)
"""
import sys
import os
import time
import json
import argparse
import threading

# Ensure runtime is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import atlas_ros2_lite as rclpy

from nodes.sensors import ImuPublisher, JointStatePublisher, ForceTorquePublisher
from nodes.control import BalanceController, GaitGenerator, JointCommander
from nodes.perception import VisionNode, ObjectDetector, SlamNode
from nodes.planning import TaskPlanner, PathPlanner
from nodes.supervisor import SupervisorNode, BrainBridgeNode


LAYERS = {
    1: ("HARDWARE (Sensors)", [
        ("ImuPublisher", lambda: ImuPublisher(rate_hz=50)),
        ("JointStatePublisher", lambda: JointStatePublisher(rate_hz=50)),
        ("ForceTorquePublisher", lambda: ForceTorquePublisher(rate_hz=50)),
    ]),
    2: ("CONTROL (Motor)", [
        ("BalanceController", lambda: BalanceController(rate_hz=100)),
        ("GaitGenerator", lambda: GaitGenerator(rate_hz=100)),
        ("JointCommander", lambda: JointCommander(rate_hz=100)),
    ]),
    3: ("PERCEPTION (Vision)", [
        ("VisionNode", lambda: VisionNode(fps=5)),
        ("ObjectDetector", lambda: ObjectDetector()),
        ("SlamNode", lambda: SlamNode()),
    ]),
    4: ("PLANNING", [
        ("TaskPlanner", lambda: TaskPlanner()),
        ("PathPlanner", lambda: PathPlanner()),
    ]),
    5: ("SUPERVISOR + BRAIN", [
        ("SupervisorNode", lambda: SupervisorNode()),
        ("BrainBridgeNode", lambda: BrainBridgeNode()),
    ]),
}


def print_banner():
    print("""
╔══════════════════════════════════════════════════════════╗
║           ATLAS SPINE — Columna Vertebral               ║
║           ROS 2 Lite Runtime (Pure Python)              ║
╠══════════════════════════════════════════════════════════╣
║  L1  Hardware   : IMU, Joints(30DOF), Force-Torque      ║
║  L2  Control    : Balance(PID/CoP), Gait, Commander     ║
║  L3  Perception : Vision, YOLO, SLAM                    ║
║  L4  Planning   : Task Planner(AI), Path Planner        ║
║  L5  Supervisor : Heartbeat, Governance, Brain Bridge   ║
╚══════════════════════════════════════════════════════════╝
""")


def topic_monitor(interval=3.0):
    """Periodically print active topics and message counts."""
    while rclpy.ok():
        time.sleep(interval)
        topics = rclpy.get_topic_list()
        if not topics:
            continue
        print(f"\n{'─'*60}")
        print(f"  TOPIC MONITOR  ({time.strftime('%H:%M:%S')})")
        print(f"{'─'*60}")
        for topic, subs in sorted(topics.items()):
            count = rclpy.get_pub_count(topic)
            latest = rclpy.get_latest_message(topic)
            preview = ""
            if latest:
                if isinstance(latest, dict):
                    data = latest.get("data")
                    if isinstance(data, str) and len(data) > 2:
                        try:
                            parsed = json.loads(data)
                            preview = json.dumps(parsed, ensure_ascii=False)[:60]
                        except Exception:
                            preview = data[:60]
                    elif "_type" in latest:
                        preview = latest["_type"]
                else:
                    preview = str(latest)[:60]
            print(f"  {topic:<40} subs={subs} msgs={count:>6}  {preview}")
        print(f"{'─'*60}")


def run_self_test(duration=5):
    """Quick self-test: start all layers, run for N seconds, report."""
    print("\n[TEST] Starting self-test...")
    rclpy.init()
    nodes = []
    for layer_id in sorted(LAYERS.keys()):
        name, factories = LAYERS[layer_id]
        for node_name, factory in factories:
            try:
                node = factory()
                nodes.append((node_name, node))
            except Exception as e:
                print(f"  [FAIL] {node_name}: {e}")

    print(f"[TEST] {len(nodes)} nodes started, running for {duration}s...")
    time.sleep(duration)

    topics = rclpy.get_topic_list()
    total_msgs = sum(rclpy.get_pub_count(t) for t in topics)
    print(f"\n[TEST] Results after {duration}s:")
    print(f"  Nodes active:  {len(nodes)}")
    print(f"  Topics active: {len(topics)}")
    print(f"  Messages sent: {total_msgs}")

    # Check critical topics
    critical = ["/atlas/imu/data", "/atlas/joint_states", "/atlas/ft/left_foot",
                "/atlas/joint_commands", "/atlas/gait/trajectory", "/atlas/hw/joint_target",
                "/atlas/supervisor/heartbeat"]
    ok_count = 0
    for t in critical:
        count = rclpy.get_pub_count(t)
        status = "OK" if count > 0 else "MISSING"
        if count > 0:
            ok_count += 1
        print(f"  {t:<45} {status} ({count} msgs)")

    print(f"\n[TEST] Score: {ok_count}/{len(critical)} critical topics active")

    for _, node in nodes:
        node.destroy_node()
    rclpy.shutdown()

    if ok_count >= 5:
        print("[TEST] PASSED")
        return 0
    else:
        print("[TEST] PARTIAL — some topics missing (may need PUSH backend running)")
        return 1


def main():
    parser = argparse.ArgumentParser(description="Atlas Spine Launcher")
    parser.add_argument("--layers", type=str, default="1,2,3,4,5",
                        help="Comma-separated layer numbers to start (default: all)")
    parser.add_argument("--monitor", action="store_true",
                        help="Enable live topic monitor")
    parser.add_argument("--test", action="store_true",
                        help="Run quick self-test (5 seconds)")
    args = parser.parse_args()

    if args.test:
        return run_self_test()

    print_banner()

    layer_ids = [int(x.strip()) for x in args.layers.split(",") if x.strip().isdigit()]

    rclpy.init()
    nodes = []

    for layer_id in sorted(layer_ids):
        if layer_id not in LAYERS:
            print(f"[WARN] Unknown layer {layer_id}, skipping")
            continue
        name, factories = LAYERS[layer_id]
        print(f"  Starting Layer {layer_id}: {name}")
        for node_name, factory in factories:
            try:
                node = factory()
                nodes.append((node_name, node))
                print(f"    + {node_name}")
            except Exception as e:
                print(f"    ! {node_name} FAILED: {e}")

    print(f"\n  Total: {len(nodes)} nodes running")
    print(f"  Press Ctrl+C to stop\n")

    if args.monitor:
        monitor_thread = threading.Thread(target=topic_monitor, daemon=True)
        monitor_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down Atlas Spine...")

    for node_name, node in nodes:
        node.destroy_node()
    rclpy.shutdown()
    print("Atlas Spine stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
