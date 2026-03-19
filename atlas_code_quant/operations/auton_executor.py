"""Autonomous execution helpers for paper-first Atlas operational tests."""
from __future__ import annotations

import importlib.util
import json
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

from api.schemas import OrderRequest
from config.settings import settings
from execution.tradier_execution import route_order_to_tradier


_DEFAULT_STATE = {
    "mode": "paper_api",
    "kill_switch_active": False,
    "last_action": None,
    "notes": "Ejecutor orientado a simulada. La automatizacion de escritorio solo esta en simulacion.",
}


class AutonExecutorService:
    def __init__(self, state_path: Path | None = None) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "auton_executor_state.json")
        self._ensure_state()

    def _ensure_state(self) -> None:
        if not self.state_path.exists():
            self._save(_DEFAULT_STATE)

    def _load(self) -> dict[str, Any]:
        self._ensure_state()
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                merged = deepcopy(_DEFAULT_STATE)
                merged.update(data)
                return merged
        except Exception:
            pass
        return deepcopy(_DEFAULT_STATE)

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        merged = deepcopy(_DEFAULT_STATE)
        merged.update(payload or {})
        self.state_path.write_text(json.dumps(merged, ensure_ascii=True, indent=2), encoding="utf-8")
        return merged

    def configure(
        self,
        *,
        mode: str | None = None,
        kill_switch_active: bool | None = None,
        notes: str | None = None,
    ) -> dict[str, Any]:
        state = self._load()
        if mode is not None:
            state["mode"] = str(mode)
        if kill_switch_active is not None:
            state["kill_switch_active"] = bool(kill_switch_active)
        if notes is not None:
            state["notes"] = str(notes)
        return self._save(state)

    def status(self) -> dict[str, Any]:
        state = self._load()
        return {
            **state,
            "supported_modes": ["disabled", "paper_api", "desktop_dry_run"],
            "pyautogui_available": importlib.util.find_spec("pyautogui") is not None,
            "failsafe_note": "PyAutoGUI supports fail-safe corners and built-in pause; desktop execution remains dry-run in this prototype.",
        }

    def emergency_stop(self, *, reason: str = "manual_stop") -> dict[str, Any]:
        state = self._load()
        state["kill_switch_active"] = True
        state["last_action"] = {
            "type": "emergency_stop",
            "reason": reason,
            "timestamp": datetime.utcnow().isoformat(),
        }
        return self._save(state)

    def clear_emergency_stop(self) -> dict[str, Any]:
        state = self._load()
        state["kill_switch_active"] = False
        state["last_action"] = {
            "type": "emergency_reset",
            "timestamp": datetime.utcnow().isoformat(),
        }
        return self._save(state)

    def execute(
        self,
        *,
        order: OrderRequest,
        action: str,
        mode: str | None = None,
    ) -> dict[str, Any]:
        state = self._load()
        effective_mode = str(mode or state.get("mode") or "disabled")
        if state.get("kill_switch_active"):
            return {
                "decision": "blocked",
                "executor_mode": effective_mode,
                "reason": "Parada de emergencia activa",
            }
        if effective_mode == "disabled" or action == "evaluate":
            decision = "evaluation_only" if action == "evaluate" else "executor_disabled"
            result = {"decision": decision, "executor_mode": effective_mode}
        elif effective_mode == "desktop_dry_run":
            result = {
                "decision": "desktop_dry_run",
                "executor_mode": effective_mode,
                "desktop_plan": self._desktop_plan(order, action=action),
            }
        elif effective_mode == "paper_api":
            routed = route_order_to_tradier(order)
            result = {
                "decision": "paper_preview_sent" if order.preview else "paper_submit_sent",
                "executor_mode": effective_mode,
                "response": routed,
            }
        else:
            result = {
                "decision": "blocked",
                "executor_mode": effective_mode,
                "reason": f"Modo de ejecucion no compatible '{effective_mode}'",
            }
        state["last_action"] = {
            "type": "execute",
            "action": action,
            "mode": effective_mode,
            "timestamp": datetime.utcnow().isoformat(),
            "decision": result.get("decision"),
        }
        self._save(state)
        return result

    @staticmethod
    def _desktop_plan(order: OrderRequest, *, action: str) -> list[dict[str, Any]]:
        return [
            {"step": 1, "action": "focus_browser", "target": "Tradier Web ticket"},
            {"step": 2, "action": "type_symbol", "value": order.symbol},
            {"step": 3, "action": "type_quantity", "value": order.size},
            {"step": 4, "action": "select_side", "value": order.side},
            {"step": 5, "action": "select_order_type", "value": order.order_type},
            {"step": 6, "action": "click_button", "value": "Previsualizar" if action == "preview" else "Confirmar"},
        ]
