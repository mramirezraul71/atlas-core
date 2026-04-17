from __future__ import annotations

from typing import Any, Dict

import requests

from ..models import AutonomyMode, Command, ModuleRisks, ModuleState
from ..module_registry import AutonomyModule


class RobotAutonomyAdapter(AutonomyModule):
    name = "robot"

    def __init__(self, base_url: str = "http://127.0.0.1:8002") -> None:
        self.base_url = base_url.rstrip("/")

    def _get(self, path: str) -> Dict[str, Any]:
        try:
            r = requests.get(f"{self.base_url}{path}", timeout=1.5)
            r.raise_for_status()
            data = r.json()
            return data if isinstance(data, dict) else {}
        except Exception as exc:
            return {"error": str(exc)}

    def _post(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        try:
            r = requests.post(f"{self.base_url}{path}", json=payload, timeout=1.5)
            r.raise_for_status()
            data = r.json()
            return data if isinstance(data, dict) else {}
        except Exception as exc:
            return {"error": str(exc)}

    def get_capabilities(self) -> Dict[str, Any]:
        return {
            "type": "robot",
            "modes": ["off", "observe", "act", "safe"],
            "supports_safe_mode": True,
        }

    def get_state(self) -> ModuleState:
        status = self._get("/status") or {}
        mode: AutonomyMode = status.get("mode", "semi")  # type: ignore[assignment]
        health = status.get("health", "ok")
        return ModuleState(
            name=self.name,
            mode=mode,
            health=str(health),
            details=status,
        )

    def get_risks(self) -> ModuleRisks:
        health = self._get("/api/health") or {}
        safe_mode_active = bool(health.get("safe_mode_active", False))
        vision_errors = health.get("vision_errors", 0)
        return ModuleRisks(
            name=self.name,
            risks={
                "safe_mode_active": safe_mode_active,
                "vision_errors": vision_errors,
            },
        )

    def apply_command(self, command: Command) -> Dict[str, Any]:
        if command.action != "set_mode":
            return {"ok": False, "reason": "unsupported_command"}

        mode = command.params.get("mode")
        if not mode:
            return {"ok": False, "reason": "missing_mode"}

        # Best-effort: map global autonomy modes to personality modes.
        # The robot backend currently exposes /personality/mode, not a direct "safe/off/act" interface.
        mapping = {
            "off": "formal",
            "observe": "casual",
            "act": "friendly",
            "safe": "formal",
        }
        personality_mode = mapping.get(str(mode), "casual")
        return self._post("/personality/mode", {"mode": personality_mode})

