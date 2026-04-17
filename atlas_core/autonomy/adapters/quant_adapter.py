from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from ..models import AutonomyMode, Command, ModuleRisks, ModuleState
from ..module_registry import AutonomyModule


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _save_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


class QuantAutonomyAdapter(AutonomyModule):
    """
    Adapter ligero que conecta el orquestador con atlas_code_quant.
    No ejecuta trades: solo ajusta flags/config de alto nivel.
    """

    name = "quant"

    def __init__(
        self,
        summary_path: Path,
        operation_state_path: Path,
    ) -> None:
        self.summary_path = summary_path
        self.operation_state_path = operation_state_path

    def _resolve_summary_path(self) -> Path:
        if self.summary_path.exists():
            return self.summary_path

        # Fallback: intenta localizar el "último" summary.json en el directorio
        parent = self.summary_path.parent
        if not parent.exists():
            return self.summary_path

        candidates = list(parent.glob("*.json"))
        if not candidates:
            return self.summary_path

        candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        return candidates[0]

    def get_capabilities(self) -> Dict[str, Any]:
        return {
            "type": "trading",
            "modes": ["manual", "semi", "auto", "safe"],
            "supports_risk_scaling": True,
        }

    def get_state(self) -> ModuleState:
        summary = _load_json(self._resolve_summary_path())
        oc_state = _load_json(self.operation_state_path)

        mode: AutonomyMode = oc_state.get("autonomy_mode", "semi")  # type: ignore[assignment]

        health = "ok"
        dd = float(summary.get("max_drawdown_pct", 0) or 0)
        if dd < -0.1:
            health = "critical"
        elif dd < -0.05:
            health = "degraded"

        return ModuleState(
            name=self.name,
            mode=mode,
            health=health,
            details={
                "win_rate_pct": summary.get("win_rate_pct"),
                "roi_pct": summary.get("roi_pct"),
                "trades": summary.get("total_trades"),
            },
        )

    def get_risks(self) -> ModuleRisks:
        summary = _load_json(self._resolve_summary_path())
        return ModuleRisks(
            name=self.name,
            risks={
                "drawdown_pct": summary.get("max_drawdown_pct"),
                "roi_pct": summary.get("roi_pct"),
            },
        )

    def apply_command(self, command: Command) -> Dict[str, Any]:
        state = _load_json(self.operation_state_path)

        if command.action == "set_mode":
            new_mode = command.params.get("mode")
            if new_mode is not None:
                state["autonomy_mode"] = new_mode

        elif command.action == "reduce_risk":
            factor = float(command.params.get("factor", 1.0))
            key = "max_risk_per_trade_pct"
            current = float(state.get(key, 0.02))
            state[key] = max(current * factor, 0.001)

        _save_json(self.operation_state_path, state)
        return {"ok": True, "updated_state": state}

