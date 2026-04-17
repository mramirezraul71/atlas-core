"""Dashboard fisiológico del Brain Core."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .state_bus import StateBus
from .workspace_bridge import WorkspaceBridge

try:
    from fastapi import APIRouter
except Exception:  # pragma: no cover - entornos sin FastAPI
    APIRouter = None  # type: ignore[assignment]


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _audit_event_count(path: Path) -> int:
    if not path.exists():
        return 0
    try:
        with path.open("r", encoding="utf-8") as fh:
            return sum(1 for _ in fh)
    except Exception:
        return 0


@dataclass
class PhysiologicalDashboard:
    state_bus: StateBus
    workspace_bridge: WorkspaceBridge
    audit_log_path: Path | None = None
    learning_root: Path | None = None

    def _knowledge_base_size(self) -> int:
        root = self.learning_root
        if root is None:
            return 0
        if not root.exists():
            return 0
        total = 0
        for p in root.glob("*"):
            if p.is_file():
                try:
                    total += p.stat().st_size
                except Exception:
                    continue
        return total

    def snapshot(self) -> dict[str, Any]:
        snap = self.state_bus.get_snapshot()
        meta = dict(snap.meta)
        beat_interval = max(0.001, _safe_float(meta.get("heartbeat_interval_sec"), 1.0))
        bpm = round(60.0 / beat_interval, 3)
        rhythm = "regular"
        if beat_interval > 3.0:
            rhythm = "irregular"
        if beat_interval > 6.0:
            rhythm = "critical"
        respiration_rate = round(max(8.0, min(24.0, bpm / 5.0)), 3)
        oxygen = round(_clamp01(0.99 - (0.01 if rhythm != "regular" else 0.0)), 3)
        co2 = round(_clamp01(0.01 + (0.02 if rhythm != "regular" else 0.0)), 3)

        decision_latency = _safe_int(meta.get("last_decision_latency_ms"), 0)
        latency_motor = max(1, int(decision_latency * 0.6)) if decision_latency > 0 else 1

        decision_quality = round(_clamp01(1.0 - (decision_latency / 1200.0)), 3)
        cerebral_activity = round(_clamp01(min(1.0, _safe_float(meta.get("event_loop_cycles"), 0.0) / 20.0)), 3)
        consciousness = round(0.98 if snap.global_mode.lower() in {"manual", "supervised"} else 0.9, 3)
        stress = round(_clamp01((1.0 - decision_quality) + (0.2 if rhythm != "regular" else 0.0)), 3)

        safety_triggers = len(meta.get("fail_safe_history", [])) if isinstance(meta.get("fail_safe_history"), list) else 0
        global_risk = "low"
        if snap.risks:
            levels = {"low": 0, "medium": 1, "high": 2, "critical": 3}
            global_risk = max(snap.risks.values(), key=lambda r: levels.get(r.level, 0)).level
        anomaly_count = _safe_int(self.workspace_bridge.metrics().get("anomalies_detected"), 0)
        event_queue_len = _safe_int(meta.get("event_queue_after"), 0)

        vision_risk = snap.risks.get("vision")
        body_state = snap.modules.get("body")
        system_health = snap.modules.get("system_health")
        threat_level = "low"
        if global_risk in {"high", "critical"}:
            threat_level = global_risk
        elif global_risk == "medium":
            threat_level = "medium"

        memory_episodes = _safe_int(meta.get("events_ingested"), 0)
        knowledge_kb = round(self._knowledge_base_size() / 1024.0, 3)

        is_alive = bpm > 0 and respiration_rate > 0 and snap.global_mode != ""
        coherence = round(
            _clamp01(
                (decision_quality * 0.35)
                + (_clamp01(1.0 - stress) * 0.25)
                + (1.0 if rhythm == "regular" else 0.4) * 0.2
                + (_clamp01(1.0 - min(1.0, event_queue_len / 25.0)) * 0.2)
            ),
            3,
        )

        return {
            "heartbeat": {
                "last_beat": meta.get("last_heartbeat_ts_utc"),
                "bpm": bpm,
                "rhythm": rhythm,
            },
            "breathing": {
                "respiration_rate": respiration_rate,
                "oxygen_saturation": oxygen,
                "co2_buildup": co2,
            },
            "nervous_system": {
                "latency_brain_ms": decision_latency,
                "latency_motor_ms": latency_motor,
                "event_queue_length": event_queue_len,
                "mode": snap.global_mode,
            },
            "brain_state": {
                "cerebral_activity": cerebral_activity,
                "consciousness_level": consciousness,
                "decision_quality": decision_quality,
                "stress_level": stress,
            },
            "sensory_input": {
                "vision": {
                    "signals": _safe_int(meta.get("events_ingested"), 0),
                    "anomalies": 1 if (vision_risk and vision_risk.level in {"high", "critical"}) else 0,
                },
                "proprioception": {
                    "health": 0.92 if body_state and body_state.health == "ok" else 0.65,
                    "risk": 0.34 if body_state and body_state.health == "ok" else 0.71,
                },
                "interoception": {
                    "system_health": 0.91 if system_health and system_health.health == "ok" else 0.63,
                },
            },
            "immune_system": {
                "threat_level": threat_level,
                "threats_detected": safety_triggers + anomaly_count,
                "healing_active": anomaly_count > 0,
            },
            "memory": {
                "episodes_recorded": memory_episodes,
                "knowledge_base_kb": knowledge_kb,
                "anomalies_learned": anomaly_count,
                "audit_events": _audit_event_count(self.audit_log_path) if self.audit_log_path else 0,
            },
            "vitals_summary": {
                "is_alive": is_alive,
                "coherence_score": coherence,
            },
        }


def create_physiology_router(dashboard: PhysiologicalDashboard):  # type: ignore[no-untyped-def]
    if APIRouter is None:  # pragma: no cover
        raise RuntimeError("FastAPI no está disponible para exponer /api/brain/physiology")
    router = APIRouter(tags=["brain"])

    @router.get("/api/brain/physiology")
    async def get_brain_physiology() -> dict[str, Any]:
        return dashboard.snapshot()

    return router

