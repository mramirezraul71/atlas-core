"""Main OpenClaw skill implementation for ATLAS."""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from .backends import AtlasRuntimeBackend, Ros2CommandBackend, ScriptBackend
from .models import DEFAULT_CONFIG, RobotBackend, SensorAdapter
from .parser import CommandValidator, NaturalLanguageParser


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    output: Dict[str, Any] = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(output.get(key), dict):
            output[key] = deep_merge(output[key], value)
        else:
            output[key] = value
    return output


def load_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    path = Path(config_path) if config_path else Path(__file__).with_name("config.json")
    if not path.exists():
        return DEFAULT_CONFIG.copy()
    loaded = json.loads(path.read_text(encoding="utf-8"))
    return deep_merge(DEFAULT_CONFIG, loaded)


class AtlasRobotSkill:
    """Main skill object to be registered inside OpenClaw."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = deep_merge(DEFAULT_CONFIG, config or {})
        self.parser = NaturalLanguageParser(self.config)
        self.validator = CommandValidator(self.config)
        self.sensors: Dict[str, SensorAdapter] = {}
        self.backend = self._build_backend()

    def _build_backend(self) -> RobotBackend:
        backend_name = str(self.config.get("execution", {}).get("backend", "atlas")).lower()
        if backend_name == "script":
            return ScriptBackend(self.config)
        if backend_name == "ros2":
            return Ros2CommandBackend(self.config)
        return AtlasRuntimeBackend(self.config)

    @property
    def skill_id(self) -> str:
        return str(self.config.get("skill", {}).get("id", "atlas_robot_skill"))

    def register_sensor(self, sensor_id: str, sensor: SensorAdapter) -> None:
        self.sensors[sensor_id] = sensor

    def sensor_snapshot(self) -> Dict[str, Any]:
        snapshot: Dict[str, Any] = {}
        for sensor_id, sensor in self.sensors.items():
            try:
                snapshot[sensor_id] = sensor.snapshot()
            except Exception as exc:
                snapshot[sensor_id] = {"ok": False, "error": str(exc)}
        return snapshot

    def run(self, text_command: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        parsed = self.parser.parse(text_command)
        validation = self.validator.validate(parsed)
        if not validation.get("allowed", False):
            emergency_result: Optional[Dict[str, Any]] = None
            if self.config.get("execution", {}).get("emergency_stop_on_violation", True):
                emergency_result = self.backend.emergency_stop(validation.get("reason", "safety_violation"))
            return {
                "ok": False,
                "status": "blocked",
                "command": parsed.__dict__,
                "validation": validation,
                "emergency_stop": emergency_result,
                "sensor_snapshot": self.sensor_snapshot(),
                "context": context or {},
            }

        result = self.backend.execute(parsed)
        return {
            "ok": bool(result.get("ok")),
            "status": "executed" if result.get("ok") else "failed",
            "command": parsed.__dict__,
            "validation": validation,
            "execution": result,
            "sensor_snapshot": self.sensor_snapshot(),
            "context": context or {},
        }

    def execute(self, text_command: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return self.run(text_command, context=context)

    def handle(self, text_command: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return self.run(text_command, context=context)


def register(config: Optional[Dict[str, Any]] = None, config_path: Optional[str] = None) -> AtlasRobotSkill:
    file_config = load_config(config_path=config_path)
    merged = deep_merge(file_config, config or {})
    return AtlasRobotSkill(config=merged)


if __name__ == "__main__":
    skill = register()
    prompt = " ".join(sys.argv[1:]).strip() or "ATLAS, mueve el brazo derecho"
    print(json.dumps(skill.run(prompt), ensure_ascii=False, indent=2))
