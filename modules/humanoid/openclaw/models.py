"""Shared models and defaults for the ATLAS OpenClaw integration."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Protocol


DEFAULT_CONFIG: Dict[str, Any] = {
    "skill": {
        "id": "atlas_robot_skill",
        "name": "ATLAS Robot Skill",
        "version": "2.0.0",
        "language": "es",
    },
    "llm": {
        "enabled": True,
        "provider": "ollama",
        "base_url": "http://127.0.0.1:11434",
        "fallback_urls": [],
        "model": "llama3.1:8b-instruct",
        "timeout_sec": 8,
    },
    "execution": {
        "backend": "atlas",
        "dry_run": False,
        "command_timeout_sec": 6,
        "emergency_stop_on_violation": True,
    },
    "paths": {
        "workspace_root": "C:/ATLAS_PUSH",
        "state_dir": "C:/ATLAS_PUSH/state/openclaw_atlas",
    },
    "atlas": {
        "default_velocity_limit": 1.0,
        "default_acceleration_limit": 2.0,
        "default_arm_step_deg": 8.0,
        "default_home_pose_deg": {
            "r_shoulder_pitch": 0.0,
            "l_shoulder_pitch": 0.0,
            "r_elbow": 20.0,
            "l_elbow": 20.0,
            "r_gripper": 1.0,
            "l_gripper": 1.0,
        },
    },
    "ros2": {
        "command_topic": "/atlas/command",
        "command_msg_type": "std_msgs/msg/String",
        "stop_topic": "/atlas/emergency_stop",
        "stop_msg_type": "std_msgs/msg/Bool",
    },
    "safety": {
        "allowed_actions": [
            "move_arm",
            "move_joint",
            "open_gripper",
            "close_gripper",
            "home_pose",
            "stop_motion",
            "emergency_stop",
        ],
        "joint_limits_deg": {
            "right_shoulder_pitch": [-90, 90],
            "right_elbow": [0, 135],
            "left_shoulder_pitch": [-90, 90],
            "left_elbow": [0, 135],
        },
        "max_single_step_deg": 40,
        "workspace_m": {
            "x": [-0.7, 0.7],
            "y": [-0.7, 0.7],
            "z": [0.0, 1.8],
        },
    },
    "sensors": {
        "rauli_vision": {
            "enabled": False,
            "adapter": "future",
            "notes": "Registrar con AtlasRobotSkill.register_sensor()",
        }
    },
}


@dataclass
class ParsedCommand:
    action: str
    target: str = ""
    params: Dict[str, Any] = field(default_factory=dict)
    source: str = "rules"
    confidence: float = 0.0
    raw_text: str = ""


class SensorAdapter(Protocol):
    """Sensor protocol for future extensions such as RAULI-VISION."""

    def snapshot(self) -> Dict[str, Any]:
        ...


class RobotBackend(Protocol):
    """Execution backend for robot actions."""

    def execute(self, command: ParsedCommand) -> Dict[str, Any]:
        ...

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        ...
