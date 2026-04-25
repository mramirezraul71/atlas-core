from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import os
from typing import Any, Literal, Mapping

from atlas_scanner.contracts import RadarDecisionHandoff, RadarSignal

GateDecision = Literal["accepted", "rejected", "caution", "bypassed"]


@dataclass(frozen=True)
class DecisionGateConfig:
    enabled: bool = False
    mode: str = "paper_supervised"
    min_structural_confidence: float = 45.0
    min_fast_pressure: float = 45.0
    max_divergence_score: float = 35.0
    allow_degraded: bool = True
    allow_structural_only: bool = True
    allow_fast_only: bool = False


@dataclass(frozen=True)
class DecisionGateEvaluation:
    timestamp: datetime
    symbol: str
    timeframe: str | None
    snapshot_classification: str
    structural_confidence_score: float | None
    fast_pressure_score: float | None
    fast_structural_alignment: str | None
    fast_structural_divergence_score: float | None
    horizon_conflict: bool | None
    cross_horizon_alignment: bool | None
    decision: GateDecision
    reason: str
    thresholds: Mapping[str, Any] = field(default_factory=dict)
    mode: str = "paper_supervised"


def config_from_env() -> DecisionGateConfig:
    return DecisionGateConfig(
        enabled=os.getenv("ATLAS_DECISION_GATE_ENABLED", "false").strip().lower() in {"1", "true", "yes"},
        mode=os.getenv("ATLAS_DECISION_GATE_MODE", "paper_supervised").strip() or "paper_supervised",
        min_structural_confidence=float(os.getenv("ATLAS_DECISION_GATE_MIN_STRUCTURAL_CONFIDENCE", "45")),
        min_fast_pressure=float(os.getenv("ATLAS_DECISION_GATE_MIN_FAST_PRESSURE", "45")),
        max_divergence_score=float(os.getenv("ATLAS_DECISION_GATE_MAX_DIVERGENCE_SCORE", "35")),
        allow_degraded=os.getenv("ATLAS_DECISION_GATE_ALLOW_DEGRADED", "true").strip().lower() in {"1", "true", "yes"},
        allow_structural_only=os.getenv("ATLAS_DECISION_GATE_ALLOW_STRUCTURAL_ONLY", "true").strip().lower() in {"1", "true", "yes"},
        allow_fast_only=os.getenv("ATLAS_DECISION_GATE_ALLOW_FAST_ONLY", "false").strip().lower() in {"1", "true", "yes"},
    )


def evaluate_handoff(handoff: RadarDecisionHandoff, config: DecisionGateConfig) -> DecisionGateEvaluation:
    signal = handoff.primary_signal
    if not config.enabled:
        return DecisionGateEvaluation(
            timestamp=datetime.now(timezone.utc),
            symbol=handoff.symbol,
            timeframe=signal.timeframe if signal is not None else None,
            snapshot_classification=_meta_str(signal, "snapshot_classification", "non_operable"),
            structural_confidence_score=_meta_float(signal, "structural_confidence_score"),
            fast_pressure_score=_meta_float(signal, "fast_pressure_score"),
            fast_structural_alignment=_meta_str(signal, "fast_structural_alignment", None),
            fast_structural_divergence_score=_meta_float(signal, "fast_structural_divergence_score"),
            horizon_conflict=_meta_bool(signal, "horizon_conflict"),
            cross_horizon_alignment=_meta_bool(signal, "cross_horizon_alignment"),
            decision="bypassed",
            reason="decision_gate_disabled",
            mode=config.mode,
            thresholds=_thresholds(config),
        )
    if signal is None:
        return DecisionGateEvaluation(
            timestamp=datetime.now(timezone.utc),
            symbol=handoff.symbol,
            timeframe=None,
            snapshot_classification="non_operable",
            structural_confidence_score=None,
            fast_pressure_score=None,
            fast_structural_alignment=None,
            fast_structural_divergence_score=None,
            horizon_conflict=None,
            cross_horizon_alignment=None,
            decision="rejected",
            reason="missing_primary_signal",
            mode=config.mode,
            thresholds=_thresholds(config),
        )
    classification = _meta_str(signal, "snapshot_classification", "non_operable")
    structural_conf = _meta_float(signal, "structural_confidence_score")
    fast_pressure = _meta_float(signal, "fast_pressure_score")
    divergence = _meta_float(signal, "fast_structural_divergence_score")
    alignment = _meta_str(signal, "fast_structural_alignment", "aligned")
    horizon_conflict = _meta_bool(signal, "horizon_conflict") or False
    cross_horizon_alignment = _meta_bool(signal, "cross_horizon_alignment")

    if classification == "non_operable":
        return _build_eval(signal, config, "rejected", "snapshot_non_operable")
    if classification == "structural_only" and not config.allow_structural_only:
        return _build_eval(signal, config, "rejected", "structural_only_not_allowed")
    if classification == "fast_only" and not config.allow_fast_only:
        return _build_eval(signal, config, "rejected", "fast_only_not_allowed")
    if classification == "operable_with_degradation" and not config.allow_degraded:
        return _build_eval(signal, config, "rejected", "degraded_not_allowed")

    if structural_conf is not None and structural_conf < config.min_structural_confidence:
        return _build_eval(signal, config, "rejected", "structural_confidence_below_threshold")
    if fast_pressure is not None and fast_pressure < config.min_fast_pressure:
        return _build_eval(signal, config, "rejected", "fast_pressure_below_threshold")
    if divergence is not None and divergence > config.max_divergence_score:
        return _build_eval(signal, config, "rejected", "divergence_above_threshold")
    if horizon_conflict and not cross_horizon_alignment:
        return _build_eval(signal, config, "caution", "horizon_conflict_detected")
    if alignment in {"fast_dominant", "structural_dominant"}:
        return _build_eval(signal, config, "caution", f"alignment_{alignment}")
    if classification in {"operable_with_degradation", "structural_only", "fast_only"}:
        return _build_eval(signal, config, "caution", f"classification_{classification}")
    return _build_eval(signal, config, "accepted", "gate_passed")


def _build_eval(signal: RadarSignal, config: DecisionGateConfig, decision: GateDecision, reason: str) -> DecisionGateEvaluation:
    return DecisionGateEvaluation(
        timestamp=datetime.now(timezone.utc),
        symbol=signal.symbol,
        timeframe=signal.timeframe,
        snapshot_classification=_meta_str(signal, "snapshot_classification", "non_operable"),
        structural_confidence_score=_meta_float(signal, "structural_confidence_score"),
        fast_pressure_score=_meta_float(signal, "fast_pressure_score"),
        fast_structural_alignment=_meta_str(signal, "fast_structural_alignment", None),
        fast_structural_divergence_score=_meta_float(signal, "fast_structural_divergence_score"),
        horizon_conflict=_meta_bool(signal, "horizon_conflict"),
        cross_horizon_alignment=_meta_bool(signal, "cross_horizon_alignment"),
        decision=decision,
        reason=reason,
        thresholds=_thresholds(config),
        mode=config.mode,
    )


def _meta_str(signal: RadarSignal | None, key: str, default: str | None) -> str | None:
    if signal is None or not isinstance(signal.meta, Mapping):
        return default
    value = signal.meta.get(key)
    if value is None:
        return default
    return str(value)


def _meta_float(signal: RadarSignal | None, key: str) -> float | None:
    if signal is None or not isinstance(signal.meta, Mapping):
        return None
    value = signal.meta.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _meta_bool(signal: RadarSignal | None, key: str) -> bool | None:
    if signal is None or not isinstance(signal.meta, Mapping):
        return None
    value = signal.meta.get(key)
    if isinstance(value, bool):
        return value
    return None


def _thresholds(config: DecisionGateConfig) -> dict[str, Any]:
    return {
        "min_structural_confidence": config.min_structural_confidence,
        "min_fast_pressure": config.min_fast_pressure,
        "max_divergence_score": config.max_divergence_score,
        "allow_degraded": config.allow_degraded,
        "allow_structural_only": config.allow_structural_only,
        "allow_fast_only": config.allow_fast_only,
    }
