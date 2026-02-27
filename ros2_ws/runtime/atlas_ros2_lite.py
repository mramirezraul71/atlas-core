"""
Atlas ROS 2 Lite Runtime
========================
Lightweight pub/sub message bus that emulates ROS 2 core APIs in pure Python.
Allows running all Atlas spine nodes WITHOUT installing ROS 2.

When ROS 2 is installed, nodes can switch to real rclpy with zero code changes
by simply importing rclpy instead of atlas_ros2_lite.

Features:
- Topic pub/sub with type-agnostic messages (dict-based)
- Node lifecycle (init, spin, shutdown)
- Parameter server (declare/get)
- Timer callbacks
- Threaded executor for multiple nodes
- Topic introspection (list topics, subscribers, publishers)
"""
from __future__ import annotations

import threading
import time
import json
import logging
import copy
from typing import Any, Callable, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from collections import defaultdict

log = logging.getLogger("atlas_ros2_lite")

# ─── Global Message Bus ───

class _MessageBus:
    """Process-wide pub/sub message bus (singleton)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._topics: Dict[str, List[Callable]] = defaultdict(list)
        self._latest: Dict[str, Any] = {}
        self._pub_count: Dict[str, int] = defaultdict(int)

    def subscribe(self, topic: str, callback: Callable):
        with self._lock:
            self._topics[topic].append(callback)

    def unsubscribe(self, topic: str, callback: Callable):
        with self._lock:
            if callback in self._topics[topic]:
                self._topics[topic].remove(callback)

    def publish(self, topic: str, msg: Any):
        with self._lock:
            self._latest[topic] = msg
            self._pub_count[topic] += 1
            callbacks = list(self._topics[topic])
        for cb in callbacks:
            try:
                cb(msg)
            except Exception as e:
                log.warning("Callback error on %s: %s", topic, e)

    def get_topics(self) -> Dict[str, int]:
        with self._lock:
            return {t: len(subs) for t, subs in self._topics.items() if subs or t in self._latest}

    def get_latest(self, topic: str) -> Any:
        with self._lock:
            return self._latest.get(topic)

    def get_pub_count(self, topic: str) -> int:
        with self._lock:
            return self._pub_count.get(topic, 0)


_BUS = _MessageBus()


# ─── Message Types (dict-based, compatible with ROS 2 field names) ───

def make_header(frame_id: str = "", stamp: float = None) -> dict:
    return {"stamp": stamp or time.time(), "frame_id": frame_id}


def make_imu(header=None, orientation=None, angular_velocity=None, linear_acceleration=None) -> dict:
    return {
        "_type": "sensor_msgs/Imu",
        "header": header or make_header(),
        "orientation": orientation or {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "angular_velocity": angular_velocity or {"x": 0.0, "y": 0.0, "z": 0.0},
        "linear_acceleration": linear_acceleration or {"x": 0.0, "y": 0.0, "z": 9.81},
    }


def make_joint_state(header=None, name=None, position=None, velocity=None, effort=None) -> dict:
    return {
        "_type": "sensor_msgs/JointState",
        "header": header or make_header(),
        "name": name or [],
        "position": position or [],
        "velocity": velocity or [],
        "effort": effort or [],
    }


def make_wrench_stamped(header=None, force=None, torque=None) -> dict:
    return {
        "_type": "geometry_msgs/WrenchStamped",
        "header": header or make_header(),
        "wrench": {
            "force": force or {"x": 0.0, "y": 0.0, "z": 0.0},
            "torque": torque or {"x": 0.0, "y": 0.0, "z": 0.0},
        },
    }


def make_string(data: str = "") -> dict:
    return {"_type": "std_msgs/String", "data": data}


def make_bool(data: bool = False) -> dict:
    return {"_type": "std_msgs/Bool", "data": data}


def make_pose_stamped(header=None, position=None, orientation=None) -> dict:
    return {
        "_type": "geometry_msgs/PoseStamped",
        "header": header or make_header(),
        "pose": {
            "position": position or {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": orientation or {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }


def make_vector3_stamped(header=None, vector=None) -> dict:
    return {
        "_type": "geometry_msgs/Vector3Stamped",
        "header": header or make_header(),
        "vector": vector or {"x": 0.0, "y": 0.0, "z": 0.0},
    }


# ─── Clock ───

class Clock:
    def now(self):
        return time.time()

    def to_msg(self):
        return time.time()


class _ClockProxy:
    def now(self):
        return _StampProxy()


class _StampProxy:
    def to_msg(self):
        return time.time()


# ─── Logger ───

class NodeLogger:
    def __init__(self, name: str):
        self._name = name
        self._log = logging.getLogger(f"atlas.{name}")

    def info(self, msg, *args, **kwargs):
        self._log.info(f"[{self._name}] {msg}", *args)

    def warn(self, msg, *args, **kwargs):
        throttle = kwargs.get("throttle_duration_sec", 0)
        self._log.warning(f"[{self._name}] {msg}", *args)

    def error(self, msg, *args, **kwargs):
        self._log.error(f"[{self._name}] {msg}", *args)

    def debug(self, msg, *args, **kwargs):
        self._log.debug(f"[{self._name}] {msg}", *args)


# ─── Publisher / Subscriber ───

class Publisher:
    def __init__(self, topic: str, msg_type=None, qos: int = 10):
        self._topic = topic
        self._msg_type = msg_type

    def publish(self, msg):
        _BUS.publish(self._topic, msg)


class Subscription:
    def __init__(self, topic: str, msg_type, callback: Callable, qos: int = 10):
        self._topic = topic
        self._callback = callback
        _BUS.subscribe(topic, callback)

    def destroy(self):
        _BUS.unsubscribe(self._topic, self._callback)


# ─── Timer ───

class Timer:
    def __init__(self, period: float, callback: Callable):
        self._period = period
        self._callback = callback
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            t0 = time.time()
            try:
                self._callback()
            except Exception as e:
                log.warning("Timer callback error: %s", e)
            elapsed = time.time() - t0
            sleep_time = max(0, self._period - elapsed)
            time.sleep(sleep_time)

    def cancel(self):
        self._running = False


# ─── Node ───

class Node:
    """Lightweight ROS 2 Node emulation."""

    def __init__(self, name: str):
        self._name = name
        self._logger = NodeLogger(name)
        self._clock = _ClockProxy()
        self._params: Dict[str, Any] = {}
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []
        self._timers: List[Timer] = []
        self._destroyed = False

    def get_name(self) -> str:
        return self._name

    def get_logger(self) -> NodeLogger:
        return self._logger

    def get_clock(self) -> _ClockProxy:
        return self._clock

    def declare_parameter(self, name: str, default_value: Any = None):
        if name not in self._params:
            self._params[name] = default_value

    def get_parameter(self, name: str):
        return _ParamValue(self._params.get(name))

    def create_publisher(self, msg_type, topic: str, qos: int = 10) -> Publisher:
        pub = Publisher(topic, msg_type, qos)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic: str, callback: Callable, qos: int = 10) -> Subscription:
        sub = Subscription(topic, msg_type, callback, qos)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period: float, callback: Callable) -> Timer:
        timer = Timer(period, callback)
        self._timers.append(timer)
        return timer

    def destroy_node(self):
        self._destroyed = True
        for t in self._timers:
            t.cancel()
        for s in self._subscriptions:
            s.destroy()
        self._timers.clear()
        self._subscriptions.clear()
        self._publishers.clear()


class _ParamValue:
    def __init__(self, value):
        self.value = value


# ─── Executor ───

_initialized = False
_shutdown_event = threading.Event()


def init(args=None):
    global _initialized
    _initialized = True
    _shutdown_event.clear()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def spin(node: Node):
    """Block until shutdown."""
    try:
        _shutdown_event.wait()
    except KeyboardInterrupt:
        pass


def spin_once(node: Node, timeout_sec: float = 0.1):
    """Process one cycle (sleep for timeout)."""
    time.sleep(timeout_sec)


def shutdown():
    global _initialized
    _initialized = False
    _shutdown_event.set()


def ok() -> bool:
    return _initialized and not _shutdown_event.is_set()


# ─── Introspection ───

def get_topic_list() -> Dict[str, int]:
    """Returns {topic_name: subscriber_count}."""
    return _BUS.get_topics()


def get_latest_message(topic: str) -> Any:
    return _BUS.get_latest(topic)


def get_pub_count(topic: str) -> int:
    return _BUS.get_pub_count(topic)
