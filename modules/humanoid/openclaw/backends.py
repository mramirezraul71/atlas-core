"""Execution backends for the ATLAS OpenClaw integration."""
from __future__ import annotations

import json
import math
import subprocess
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

from modules.humanoid.cortex.parietal.body_schema import BodySchema
from modules.humanoid.governance.state import get_emergency_stop, set_emergency_stop
from modules.humanoid.medulla.bus import get_medulla
from modules.humanoid.medulla.schemas import GripperCommand, JointPositionCommand, SystemAlert

from .models import ParsedCommand


ARM_TO_JOINT = {
    "right": "r_shoulder_pitch",
    "left": "l_shoulder_pitch",
}

GRIPPER_TO_JOINT = {
    "right": "r_gripper",
    "left": "l_gripper",
}

JOINT_ALIASES = {
    "right_shoulder_pitch": "r_shoulder_pitch",
    "left_shoulder_pitch": "l_shoulder_pitch",
    "right_elbow": "r_elbow",
    "left_elbow": "l_elbow",
    "r_shoulder_pitch": "r_shoulder_pitch",
    "l_shoulder_pitch": "l_shoulder_pitch",
    "r_elbow": "r_elbow",
    "l_elbow": "l_elbow",
    "r_gripper": "r_gripper",
    "l_gripper": "l_gripper",
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _deg_to_rad(value: float) -> float:
    return math.radians(value)


def _rad_to_deg(value: float) -> float:
    return math.degrees(value)


class Ros2CommandBackend:
    """ROS2 backend using `ros2 topic pub` shell commands."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.exec_cfg = config.get("execution", {})
        self.ros2_cfg = config.get("ros2", {})

    def _run(self, command: list[str]) -> Dict[str, Any]:
        if self.exec_cfg.get("dry_run", False):
            return {"ok": True, "dry_run": True, "command": command}
        try:
            proc = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=int(self.exec_cfg.get("command_timeout_sec", 6)),
                check=False,
            )
            return {
                "ok": proc.returncode == 0,
                "returncode": proc.returncode,
                "stdout": (proc.stdout or "").strip(),
                "stderr": (proc.stderr or "").strip(),
                "command": command,
            }
        except Exception as exc:
            return {"ok": False, "error": str(exc), "command": command}

    def execute(self, command: ParsedCommand) -> Dict[str, Any]:
        if command.action == "emergency_stop":
            return self.emergency_stop(command.params.get("reason", "manual_emergency_stop"))
        topic = self.ros2_cfg.get("command_topic", "/atlas/command")
        msg_type = self.ros2_cfg.get("command_msg_type", "std_msgs/msg/String")
        payload = {
            "action": command.action,
            "target": command.target,
            "params": command.params,
            "source": command.source,
        }
        msg = json.dumps(payload, ensure_ascii=False)
        ros_msg = json.dumps({"data": msg}, ensure_ascii=False)
        return self._run(["ros2", "topic", "pub", "--once", topic, msg_type, ros_msg])

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        topic = self.ros2_cfg.get("stop_topic", "/atlas/emergency_stop")
        msg_type = self.ros2_cfg.get("stop_msg_type", "std_msgs/msg/Bool")
        result = self._run(["ros2", "topic", "pub", "--once", topic, msg_type, "{data: true}"])
        result["reason"] = reason
        return result


class ScriptBackend:
    """Python callback backend for test environments or custom dispatchers."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.exec_cfg = config.get("execution", {})
        self.callbacks: Dict[str, Any] = {}

    def register_callback(self, action: str, callback: Any) -> None:
        self.callbacks[action] = callback

    def execute(self, command: ParsedCommand) -> Dict[str, Any]:
        if command.action == "emergency_stop":
            return self.emergency_stop(command.params.get("reason", "manual_emergency_stop"))
        if self.exec_cfg.get("dry_run", False):
            return {
                "ok": True,
                "dry_run": True,
                "action": command.action,
                "target": command.target,
                "params": command.params,
            }
        fn = self.callbacks.get(command.action)
        if fn is None:
            return {
                "ok": True,
                "action": command.action,
                "target": command.target,
                "params": command.params,
                "note": "No callback configured; command accepted for external dispatcher.",
            }
        try:
            return {"ok": True, "result": fn(target=command.target, **command.params)}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        fn = self.callbacks.get("emergency_stop")
        if fn is None:
            return {"ok": True, "note": "Emergency stop propagated.", "reason": reason}
        try:
            return {"ok": True, "result": fn(reason=reason), "reason": reason}
        except Exception as exc:
            return {"ok": False, "error": str(exc), "reason": reason}


class AtlasRuntimeBackend:
    """Backend that talks directly to ATLAS runtime primitives."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.exec_cfg = config.get("execution", {})
        self.paths_cfg = config.get("paths", {})
        self.atlas_cfg = config.get("atlas", {})
        self.medulla = get_medulla()
        if not getattr(self.medulla, "_running", False):
            self.medulla.start()
        self.body_schema = BodySchema()
        self.state_file = self._resolve_state_file()
        self._load_state()

    def _resolve_state_file(self) -> Path:
        state_dir = Path(self.paths_cfg.get("state_dir", "C:/ATLAS_PUSH/state/openclaw_atlas"))
        state_dir.mkdir(parents=True, exist_ok=True)
        return state_dir / "runtime_state.json"

    def _load_state(self) -> None:
        if not self.state_file.exists():
            return
        try:
            payload = json.loads(self.state_file.read_text(encoding="utf-8"))
            joints = payload.get("joints", {})
            if isinstance(joints, dict):
                cleaned = {str(key): float(value) for key, value in joints.items()}
                self.body_schema.set_joint_positions(cleaned, validate=False)
        except Exception:
            return

    def _persist_state(self, command: ParsedCommand, result: Dict[str, Any]) -> None:
        payload = {
            "updated_at": _utc_now(),
            "joints": self.body_schema.get_joint_positions(),
            "summary": self.body_schema.to_dict(),
            "last_command": {
                "action": command.action,
                "target": command.target,
                "params": command.params,
                "source": command.source,
            },
            "last_result": result,
        }
        self.state_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _publish_alert(self, level: str, message: str, data: Dict[str, Any] | None = None) -> None:
        alert = SystemAlert(
            alert_id=f"openclaw-{uuid.uuid4()}",
            level=level,
            source="openclaw.atlas",
            message=message,
            data=data or {},
        )
        self.medulla.publish_alert(alert)

    def _set_joint_position(self, joint_id: str, target_rad: float) -> Dict[str, Any]:
        errors = self.body_schema.set_joint_positions({joint_id: target_rad}, validate=True)
        if errors:
            return {"ok": False, "errors": errors}
        arm_id = "right" if joint_id.startswith("r_") else "left"
        command = JointPositionCommand(
            joint_id=joint_id,
            position=target_rad,
            velocity_limit=float(self.atlas_cfg.get("default_velocity_limit", 1.0)),
            acceleration_limit=float(self.atlas_cfg.get("default_acceleration_limit", 2.0)),
        )
        self.medulla.send_arm_command(arm_id, joint_id, command)
        return {"ok": True, "joint_id": joint_id, "target_rad": target_rad, "target_deg": _rad_to_deg(target_rad)}

    def _move_arm(self, side: str, step_deg: float) -> Dict[str, Any]:
        joint_id = ARM_TO_JOINT[side]
        current_rad = float(self.body_schema.get_joint_positions().get(joint_id, 0.0))
        target_rad = current_rad + _deg_to_rad(step_deg)
        result = self._set_joint_position(joint_id, target_rad)
        if result.get("ok"):
            result["step_deg"] = step_deg
            result["previous_deg"] = _rad_to_deg(current_rad)
        return result

    def _move_joint(self, joint_name: str, angle_deg: float) -> Dict[str, Any]:
        joint_id = JOINT_ALIASES.get(joint_name, joint_name)
        if joint_id not in self.body_schema.joints:
            return {"ok": False, "error": f"unknown_joint:{joint_name}"}
        if joint_id in GRIPPER_TO_JOINT.values():
            return self._set_gripper_position(joint_id, 1.0 if angle_deg > 0 else 0.0)
        return self._set_joint_position(joint_id, _deg_to_rad(angle_deg))

    def _set_gripper_position(self, joint_id: str, position: float) -> Dict[str, Any]:
        safe_position = max(0.0, min(1.0, position))
        errors = self.body_schema.set_joint_positions({joint_id: safe_position}, validate=True)
        if errors:
            return {"ok": False, "errors": errors}
        gripper_side = "right" if joint_id == "r_gripper" else "left"
        self.medulla.send_gripper_command(
            gripper_side,
            GripperCommand(
                gripper_id=joint_id,
                action="position",
                position=safe_position,
                speed=0.5,
            ),
        )
        return {"ok": True, "gripper": gripper_side, "position": safe_position}

    def _move_gripper(self, side: str, open_state: bool) -> Dict[str, Any]:
        sides = ("right", "left") if side == "both" else (side,)
        items = []
        for current_side in sides:
            joint_id = GRIPPER_TO_JOINT[current_side]
            items.append(self._set_gripper_position(joint_id, 1.0 if open_state else 0.0))
        ok = all(item.get("ok") for item in items)
        return {"ok": ok, "items": items}

    def _home_pose(self) -> Dict[str, Any]:
        pose = dict(self.atlas_cfg.get("default_home_pose_deg", {}))
        items = []
        for joint_id, value in pose.items():
            if joint_id in GRIPPER_TO_JOINT.values():
                gripper_value = float(value)
                if gripper_value > 1.0:
                    gripper_value = gripper_value / 100.0
                items.append(self._set_gripper_position(joint_id, gripper_value))
            else:
                items.append(self._set_joint_position(joint_id, _deg_to_rad(float(value))))
        ok = all(item.get("ok") for item in items)
        return {"ok": ok, "items": items}

    def execute(self, command: ParsedCommand) -> Dict[str, Any]:
        if command.action == "emergency_stop":
            return self.emergency_stop(command.params.get("reason", "manual_emergency_stop"))
        if get_emergency_stop():
            return {"ok": False, "error": "emergency_stop_active"}
        if self.exec_cfg.get("dry_run", False):
            return {
                "ok": True,
                "dry_run": True,
                "action": command.action,
                "target": command.target,
                "params": command.params,
            }

        if command.action == "move_arm":
            result = self._move_arm(command.target, float(command.params.get("angle_deg", 0.0)))
        elif command.action == "move_joint":
            result = self._move_joint(str(command.params.get("joint", "")), float(command.params.get("angle_deg", 0.0)))
        elif command.action == "open_gripper":
            result = self._move_gripper(command.target or "both", True)
        elif command.action == "close_gripper":
            result = self._move_gripper(command.target or "both", False)
        elif command.action == "home_pose":
            result = self._home_pose()
        elif command.action == "stop_motion":
            self._publish_alert("warning", "OpenClaw solicito stop_motion", {"source": command.source})
            result = {"ok": True, "stopped": True}
        else:
            result = {"ok": False, "error": f"unsupported_action:{command.action}"}

        self._persist_state(command, result)
        return result

    def emergency_stop(self, reason: str) -> Dict[str, Any]:
        set_emergency_stop(True, reason=reason, actor="openclaw")
        self._publish_alert("emergency", "Emergency stop activado desde OpenClaw", {"reason": reason})
        result = {"ok": True, "emergency_stop": True, "reason": reason}
        self._persist_state(ParsedCommand(action="emergency_stop", params={"reason": reason}), result)
        return result
