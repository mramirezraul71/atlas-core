"""
autotune.py — Autoajuste híbrido de configuración para Radar.

Diseño:
- Reglas deterministas (siempre activas) para seguridad.
- Modo manual/asistido/auto.
- Rollback transaccional del último parche aplicado.
- Scorer especializado opcional (XGBoost) con fallback heurístico.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from pydantic import BaseModel, Field

from .state.journal import Journal


class AutoTuneConfig(BaseModel):
    mode: str = Field(default="manual")  # manual | assisted | auto
    enabled: bool = Field(default=False)
    interval_seconds: int = Field(default=900, ge=30, le=86400)
    observe_window_seconds: int = Field(default=600, ge=60, le=86400)
    # Guardrails de paso por ajuste
    step_edge_net: float = Field(default=0.0025, ge=0.0001, le=0.05)
    step_edge_threshold: float = Field(default=0.0020, ge=0.0001, le=0.05)
    step_kelly: float = Field(default=0.05, ge=0.01, le=0.5)
    # Límites duros
    min_edge_net: float = Field(default=0.0, ge=0.0, le=0.5)
    max_edge_net: float = Field(default=0.2, ge=0.0, le=0.5)
    min_edge_threshold: float = Field(default=0.005, ge=0.0, le=0.95)
    max_edge_threshold: float = Field(default=0.2, ge=0.0, le=0.95)
    min_kelly: float = Field(default=0.05, ge=0.01, le=1.0)
    max_kelly: float = Field(default=0.5, ge=0.01, le=1.0)
    degrade_drawdown_cents: int = Field(default=1500, ge=100)
    degrade_loss_streak: int = Field(default=4, ge=1, le=50)


class AutoTunePatch(BaseModel):
    edge_threshold: Optional[float] = None
    edge_net_min: Optional[float] = None
    kelly_fraction: Optional[float] = None
    confidence_min: Optional[float] = None
    max_open_positions: Optional[int] = None
    reason: str = ""
    confidence: float = 0.0


@dataclass
class AutoTuneState:
    last_run_ts: float = 0.0
    last_apply_ts: float = 0.0
    status: str = "idle"  # idle | evaluating | suggested | applied | rollback | hold
    last_reason: str = ""
    pending_patch: Optional[AutoTunePatch] = None
    last_applied_patch: Optional[AutoTunePatch] = None
    rollback_snapshot: dict[str, Any] = field(default_factory=dict)
    metrics_before_apply: dict[str, Any] = field(default_factory=dict)
    metrics_after_window: dict[str, Any] = field(default_factory=dict)
    last_score: float = 0.0
    scorer: str = "heuristic"


class SpecializedScorer:
    """Scorer tabular opcional: XGBoost si está disponible."""

    def __init__(self) -> None:
        self._model = None
        self.name = "heuristic"
        try:
            from xgboost import XGBRegressor  # type: ignore

            self._model = XGBRegressor(
                n_estimators=64,
                max_depth=4,
                learning_rate=0.08,
                subsample=0.8,
                colsample_bytree=0.8,
                objective="reg:squarederror",
                random_state=42,
            )
            self.name = "xgboost"
        except Exception:
            self._model = None

    @staticmethod
    def _heuristic_score(features: dict[str, float]) -> float:
        # score>0 favorece aplicar
        pnl = features.get("pnl_net_usd", 0.0)
        dd = features.get("drawdown_usd", 0.0)
        hit = features.get("hit_rate", 0.0)
        losses = features.get("consecutive_losses", 0.0)
        actionable = features.get("actionable_pct", 0.0)
        return (
            -0.08 * pnl
            - 0.45 * dd
            + 0.60 * hit
            - 0.25 * losses
            + 0.10 * actionable
        )

    def score(self, features: dict[str, float]) -> float:
        if self._model is None:
            return self._heuristic_score(features)
        # En esta fase usamos xgboost en modo shadow sin entrenamiento persistente.
        return self._heuristic_score(features)


class AutoTuneController:
    def __init__(self, log_dir: Path, cfg: Optional[AutoTuneConfig] = None) -> None:
        self.cfg = cfg or AutoTuneConfig()
        self.state = AutoTuneState()
        self.journal = Journal(log_dir)
        self.scorer = SpecializedScorer()
        self.state.scorer = self.scorer.name

    def configure(self, **kwargs: Any) -> AutoTuneConfig:
        self.cfg = self.cfg.model_copy(update=kwargs)
        self.journal.write(
            "autotune",
            {
                "event": "config_update",
                "config": self.cfg.model_dump(mode="json"),
            },
        )
        return self.cfg

    def status(self) -> dict[str, Any]:
        return {
            "config": self.cfg.model_dump(mode="json"),
            "state": {
                "last_run_ts": self.state.last_run_ts,
                "last_apply_ts": self.state.last_apply_ts,
                "status": self.state.status,
                "last_reason": self.state.last_reason,
                "last_score": self.state.last_score,
                "scorer": self.state.scorer,
                "pending_patch": self.state.pending_patch.model_dump(mode="json")
                if self.state.pending_patch
                else None,
                "last_applied_patch": self.state.last_applied_patch.model_dump(mode="json")
                if self.state.last_applied_patch
                else None,
                "metrics_before_apply": self.state.metrics_before_apply,
                "metrics_after_window": self.state.metrics_after_window,
            },
        }

    @staticmethod
    def _extract_features(metrics: dict[str, Any]) -> dict[str, float]:
        perf = dict(metrics.get("performance", {}) or {})
        risk = dict(metrics.get("risk", {}) or {})
        return {
            "pnl_net_usd": float(perf.get("pnl_net_cents", 0.0)) / 100.0,
            "drawdown_usd": float(perf.get("current_drawdown_cents", 0.0)) / 100.0,
            "hit_rate": float(perf.get("hit_rate", 0.0)),
            "expectancy_usd": float(perf.get("expectancy_cents", 0.0)) / 100.0,
            "consecutive_losses": float(risk.get("consecutive_losses", 0.0)),
            "actionable_pct": float(metrics.get("actionable_pct", 0.0)),
        }

    def _bounded(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _propose_patch(self, runtime: dict[str, Any], metrics: dict[str, Any]) -> AutoTunePatch:
        features = self._extract_features(metrics)
        score = self.scorer.score(features)
        self.state.last_score = float(score)

        perf = dict(metrics.get("performance", {}) or {})
        risk = dict(metrics.get("risk", {}) or {})
        losses = int(risk.get("consecutive_losses", 0))
        dd_cents = int(perf.get("current_drawdown_cents", 0))
        hit = float(perf.get("hit_rate", 0.0))
        exp_c = float(perf.get("expectancy_cents", 0.0))

        edge_thr = float(runtime.get("edge_threshold", 0.05))
        edge_net = float(runtime.get("edge_net_min", 0.03))
        kelly = float(runtime.get("kelly_fraction", 0.25))

        reason = "hold"
        confidence = 0.5
        patch = AutoTunePatch(reason=reason, confidence=confidence)

        # Reacción a pérdida: endurece edge net y baja Kelly.
        if losses >= self.cfg.degrade_loss_streak or dd_cents >= self.cfg.degrade_drawdown_cents:
            reason = f"loss_reaction(losses={losses},dd={dd_cents})"
            patch.edge_net_min = self._bounded(
                edge_net + self.cfg.step_edge_net,
                self.cfg.min_edge_net,
                self.cfg.max_edge_net,
            )
            patch.kelly_fraction = self._bounded(
                kelly - self.cfg.step_kelly,
                self.cfg.min_kelly,
                self.cfg.max_kelly,
            )
            patch.edge_threshold = self._bounded(
                edge_thr + self.cfg.step_edge_threshold,
                self.cfg.min_edge_threshold,
                self.cfg.max_edge_threshold,
            )
            confidence = 0.85
        # Mejora sostenida: libera ligeramente para recuperar oportunidad.
        elif hit >= 0.55 and exp_c > 0:
            reason = "performance_expand"
            patch.edge_net_min = self._bounded(
                edge_net - self.cfg.step_edge_net,
                self.cfg.min_edge_net,
                self.cfg.max_edge_net,
            )
            patch.kelly_fraction = self._bounded(
                kelly + self.cfg.step_kelly,
                self.cfg.min_kelly,
                self.cfg.max_kelly,
            )
            confidence = 0.70
        else:
            reason = "no_change_window"
            confidence = 0.55

        patch.reason = reason
        patch.confidence = confidence
        return patch

    def _snapshot_runtime(self, runtime: dict[str, Any]) -> dict[str, Any]:
        return {
            "edge_threshold": float(runtime.get("edge_threshold", 0.05)),
            "edge_net_min": float(runtime.get("edge_net_min", 0.03)),
            "kelly_fraction": float(runtime.get("kelly_fraction", 0.25)),
            "confidence_min": float(runtime.get("confidence_min", 0.55)),
            "max_open_positions": int(runtime.get("max_open_positions", 10)),
        }

    def _should_run(self) -> bool:
        if not self.cfg.enabled:
            return False
        now = time.time()
        return (now - self.state.last_run_ts) >= self.cfg.interval_seconds

    def propose(self, runtime: dict[str, Any], metrics: dict[str, Any]) -> AutoTunePatch:
        self.state.status = "evaluating"
        self.state.last_run_ts = time.time()
        patch = self._propose_patch(runtime, metrics)
        self.state.pending_patch = patch
        self.state.last_reason = patch.reason
        self.state.status = "suggested"
        self.journal.write(
            "autotune",
            {
                "event": "proposal",
                "reason": patch.reason,
                "confidence": patch.confidence,
                "patch": patch.model_dump(mode="json"),
                "features": self._extract_features(metrics),
                "score": self.state.last_score,
            },
        )
        return patch

    def maybe_autorun(self, runtime: dict[str, Any], metrics: dict[str, Any]) -> Optional[AutoTunePatch]:
        if not self._should_run():
            return None
        return self.propose(runtime, metrics)

    def can_apply(self, patch: AutoTunePatch) -> bool:
        # En asistido/auto exigimos confianza mínima.
        return patch.confidence >= 0.6 and patch.reason != "no_change_window"

    def mark_applied(self, patch: AutoTunePatch, runtime_before: dict[str, Any], metrics_before: dict[str, Any]) -> None:
        self.state.rollback_snapshot = self._snapshot_runtime(runtime_before)
        self.state.metrics_before_apply = self._extract_features(metrics_before)
        self.state.last_applied_patch = patch
        self.state.last_apply_ts = time.time()
        self.state.status = "applied"
        self.journal.write(
            "autotune",
            {
                "event": "apply",
                "patch": patch.model_dump(mode="json"),
                "runtime_before": self.state.rollback_snapshot,
                "features_before": self.state.metrics_before_apply,
            },
        )

    def evaluate_post_window(self, metrics_now: dict[str, Any]) -> bool:
        """True => mantener, False => rollback."""
        before = self.state.metrics_before_apply or {}
        after = self._extract_features(metrics_now)
        self.state.metrics_after_window = after
        # Degradación simple: más DD y expectativa peor.
        dd_before = float(before.get("drawdown_usd", 0.0))
        dd_after = float(after.get("drawdown_usd", 0.0))
        exp_before = float(before.get("expectancy_usd", 0.0))
        exp_after = float(after.get("expectancy_usd", 0.0))
        keep = not ((dd_after - dd_before) > 5.0 and exp_after < exp_before)
        self.journal.write(
            "autotune",
            {
                "event": "post_window_eval",
                "keep": keep,
                "features_before": before,
                "features_after": after,
            },
        )
        return keep

    def rollback_payload(self) -> dict[str, Any]:
        snap = dict(self.state.rollback_snapshot or {})
        return {
            "edge_threshold": snap.get("edge_threshold"),
            "edge_net_min": snap.get("edge_net_min"),
            "kelly_fraction": snap.get("kelly_fraction"),
            "confidence_min": snap.get("confidence_min"),
            "max_open_positions": snap.get("max_open_positions"),
        }

    def mark_rollback(self, reason: str) -> None:
        self.state.status = "rollback"
        self.state.last_reason = reason
        self.journal.write(
            "autotune",
            {
                "event": "rollback",
                "reason": reason,
                "snapshot": self.state.rollback_snapshot,
            },
        )

    def export_report(self, out_path: Path) -> Path:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        rows = self.journal.read("autotune", limit=2000)
        report = {
            "generated_ts": time.time(),
            "config": self.cfg.model_dump(mode="json"),
            "state": self.status(),
            "events": rows,
        }
        out_path.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
        return out_path

