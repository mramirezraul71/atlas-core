from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import requests

try:
    from .config import VisionSensorConfig
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig


@dataclass(slots=True)
class FusionDecision:
    confidence: float
    action: str
    primary_source: str
    agreement_count: int
    contradictions: list[str] = field(default_factory=list)
    reasons: list[str] = field(default_factory=list)
    payload: dict[str, Any] = field(default_factory=dict)


class TelemetrySnapshotProvider:
    def __init__(self, config: VisionSensorConfig, logger: Any) -> None:
        self.config = config
        self.logger = logger
        self._cache: dict[str, Any] = {}
        self._cache_ts = 0.0

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        if self._cache and (now - self._cache_ts) < self.config.telemetry_cache_ttl_sec:
            return self._cache

        payload = {
            "captured_at": time.time(),
            "system": self._system_health(),
            "quant": self._quant_status(),
            "quant_state": self._quant_state_file(),
        }
        self._cache = payload
        self._cache_ts = now
        return payload

    def _system_health(self) -> dict[str, Any]:
        try:
            response = requests.get(self.config.push_health_url, timeout=self.config.telemetry_timeout_sec)
            body = response.json() if response.headers.get("content-type", "").startswith("application/json") else {}
            return {
                "ok": bool(response.ok),
                "status_code": response.status_code,
                "service": body.get("service"),
                "payload": body,
            }
        except Exception as exc:
            return {"ok": False, "status_code": 0, "error": str(exc)}

    def _quant_status(self) -> dict[str, Any]:
        for url in self.config.quant_status_urls:
            try:
                response = requests.get(url, timeout=self.config.telemetry_timeout_sec)
                body = response.json() if response.content else {}
                if response.ok and isinstance(body, dict):
                    return {"ok": True, "status_code": response.status_code, "url": url, "payload": body}
            except Exception as exc:
                last_error = str(exc)
                continue
        return {"ok": False, "status_code": 0, "payload": {}, "error": locals().get("last_error", "quant_unavailable")}

    def _quant_state_file(self) -> dict[str, Any]:
        path = Path(self.config.quant_state_path)
        if not path.exists():
            return {"ok": False, "path": str(path), "reason": "missing"}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            return {"ok": isinstance(payload, dict), "path": str(path), "payload": payload if isinstance(payload, dict) else {}}
        except Exception as exc:
            return {"ok": False, "path": str(path), "reason": str(exc)}


class MultimodalFusion:
    def __init__(self, config: VisionSensorConfig, logger: Any) -> None:
        self.config = config
        self.logger = logger
        self.telemetry = TelemetrySnapshotProvider(config, logger)

    def snapshot(self) -> dict[str, Any]:
        return self.telemetry.snapshot()

    def fuse(
        self,
        *,
        visual_result: dict[str, Any],
        candle_result: dict[str, Any],
        geometry_result: dict[str, Any],
        telemetry_snapshot: dict[str, Any],
    ) -> FusionDecision:
        reasons: list[str] = []
        contradictions: list[str] = []

        chart_bias = str((visual_result.get("payload") or {}).get("chart_bias") or "unknown")
        visual_pattern = str(visual_result.get("pattern") or "")
        candle_direction = str((candle_result.get("payload") or {}).get("top_direction") or "unknown")
        geometry_pattern = str(((geometry_result.get("hits") or [{}])[0] or {}).get("pattern") or "")

        quant_payload = ((telemetry_snapshot.get("quant") or {}).get("payload") or {})
        state_payload = ((telemetry_snapshot.get("quant_state") or {}).get("payload") or {})
        system_ok = bool((telemetry_snapshot.get("system") or {}).get("ok"))
        reconciliation_failed = bool(state_payload.get("last_operational_error")) or bool(state_payload.get("fail_safe_active"))
        visual_conf = float(visual_result.get("confidence") or 0.0)
        candle_conf = float(candle_result.get("confidence") or 0.0)
        geometry_conf = float(geometry_result.get("confidence") or 0.0)

        agreement_count = 0
        if chart_bias in {"bullish", "bearish"} and candle_direction == chart_bias:
            agreement_count += 1
            reasons.append(f"vision y TA-Lib coinciden en sesgo {chart_bias}")
        if chart_bias in {"bullish", "bearish"} and geometry_pattern in {"channel", "triangle"}:
            agreement_count += 1
            reasons.append("la geometria confirma estructura compatible con el sesgo visual")
        if quant_payload and state_payload.get("last_decision", {}).get("allowed") is True:
            agreement_count += 1
            reasons.append("la telemetria de Quant no muestra bloqueo operacional en el ultimo ciclo")

        if chart_bias in {"bullish", "bearish"} and candle_direction in {"bullish", "bearish"} and candle_direction != chart_bias:
            contradictions.append("TA-Lib y vision discrepan en direccion")
        if not system_ok:
            contradictions.append("ATLAS PUSH no reporta health sana")
        if reconciliation_failed:
            contradictions.append("Quant muestra error operacional o fail-safe activo")

        base_confidence = max(visual_conf, candle_conf, geometry_conf)
        confidence = base_confidence
        if agreement_count >= 2:
            confidence = min(1.0, confidence + 0.18)
        elif agreement_count == 1:
            confidence = min(1.0, confidence + 0.06)
        if contradictions:
            confidence = max(0.0, confidence - min(0.12 * len(contradictions), 0.36))

        action = "observe"
        if contradictions:
            action = "degrade"
        if not system_ok or reconciliation_failed:
            action = "degrade"
        if agreement_count >= 2 and confidence >= 0.7 and not contradictions:
            action = "confirm"

        primary_source = "vision"
        source_scores = {
            "vision": visual_conf,
            "talib": candle_conf,
            "geometry": geometry_conf,
            "telemetry": 0.7 if quant_payload else 0.0,
        }
        primary_source = max(source_scores, key=source_scores.get)

        payload = {
            "chart_bias": chart_bias,
            "visual_pattern": visual_pattern,
            "candle_top_pattern": (candle_result.get("payload") or {}).get("top_pattern"),
            "candle_direction": candle_direction,
            "geometry_pattern": geometry_pattern,
            "source_scores": {key: round(val, 4) for key, val in source_scores.items()},
            "system_ok": system_ok,
            "quant_status_url": (telemetry_snapshot.get("quant") or {}).get("url"),
            "quant_last_decision": state_payload.get("last_decision") or {},
            "quant_last_operational_error": state_payload.get("last_operational_error") or {},
        }
        if not reasons:
            reasons.append("evidencia insuficiente para elevar la confianza")

        return FusionDecision(
            confidence=round(confidence, 4),
            action=action,
            primary_source=primary_source,
            agreement_count=agreement_count,
            contradictions=contradictions,
            reasons=reasons,
            payload=payload,
        )
