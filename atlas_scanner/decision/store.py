from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from atlas_scanner.decision.gate import DecisionGateEvaluation


@dataclass(frozen=True)
class DecisionGateRecord:
    timestamp: str
    symbol: str
    timeframe: str | None
    snapshot_classification: str
    structural_confidence_score: float | None
    fast_pressure_score: float | None
    fast_structural_alignment: str | None
    fast_structural_divergence_score: float | None
    horizon_conflict: bool | None
    cross_horizon_alignment: bool | None
    decision: str
    reason: str
    thresholds: dict[str, Any]
    mode: str


class JsonlDecisionGateStore:
    def __init__(
        self,
        path: Path,
        *,
        max_records: int = 10000,
        max_bytes: int = 8_000_000,
    ) -> None:
        self.path = path
        self.max_records = max(100, int(max_records))
        self.max_bytes = max(100_000, int(max_bytes))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.path.write_text("", encoding="utf-8")

    def append(self, evaluation: DecisionGateEvaluation) -> DecisionGateRecord:
        self._rotate_if_needed(incoming_records=1)
        record = _from_eval(evaluation)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(_to_json(record), ensure_ascii=False))
            handle.write("\n")
        return record

    def recent(
        self,
        *,
        limit: int = 100,
        symbol: str | None = None,
    ) -> tuple[DecisionGateRecord, ...]:
        lines = self.path.read_text(encoding="utf-8").splitlines()
        out: list[DecisionGateRecord] = []
        for raw in reversed(lines):
            if not raw.strip():
                continue
            payload = json.loads(raw)
            record = _from_json(payload)
            if symbol and record.symbol.upper() != symbol.upper():
                continue
            out.append(record)
            if len(out) >= max(1, limit):
                break
        return tuple(out)

    def replay(
        self,
        *,
        symbol: str,
        limit: int = 200,
    ) -> tuple[DecisionGateRecord, ...]:
        return self.recent(limit=limit, symbol=symbol)

    def stats(self) -> dict[str, Any]:
        lines = self.path.read_text(encoding="utf-8").splitlines() if self.path.exists() else []
        records = [_from_json(json.loads(raw)) for raw in lines if raw.strip()]
        decisions = {"accepted": 0, "rejected": 0, "caution": 0, "bypassed": 0}
        reasons: dict[str, int] = {}
        classes: dict[str, int] = {}
        structural_scores: list[float] = []
        fast_scores: list[float] = []
        for item in records:
            decisions[item.decision] = decisions.get(item.decision, 0) + 1
            reasons[item.reason] = reasons.get(item.reason, 0) + 1
            classes[item.snapshot_classification] = classes.get(item.snapshot_classification, 0) + 1
            if isinstance(item.structural_confidence_score, (int, float)):
                structural_scores.append(float(item.structural_confidence_score))
            if isinstance(item.fast_pressure_score, (int, float)):
                fast_scores.append(float(item.fast_pressure_score))
        return {
            "path": str(self.path),
            "total_decisions": len(records),
            "accepted_count": decisions.get("accepted", 0),
            "rejected_count": decisions.get("rejected", 0),
            "caution_count": decisions.get("caution", 0),
            "bypassed_count": decisions.get("bypassed", 0),
            "rejection_reasons_breakdown": reasons,
            "classification_breakdown": classes,
            "avg_structural_confidence_score": round(sum(structural_scores) / len(structural_scores), 4) if structural_scores else None,
            "avg_fast_pressure_score": round(sum(fast_scores) / len(fast_scores), 4) if fast_scores else None,
            "file_size_bytes": self.path.stat().st_size if self.path.exists() else 0,
            "record_count": len(records),
            "max_records": self.max_records,
            "max_bytes": self.max_bytes,
        }

    def _rotate_if_needed(self, *, incoming_records: int) -> None:
        if not self.path.exists():
            return
        lines = self.path.read_text(encoding="utf-8").splitlines()
        size_bytes = self.path.stat().st_size
        projected_records = len(lines) + incoming_records
        if projected_records <= self.max_records and size_bytes <= self.max_bytes:
            return
        keep_from = max(0, len(lines) - (self.max_records // 2))
        kept = lines[keep_from:]
        with self.path.open("w", encoding="utf-8") as handle:
            if kept:
                handle.write("\n".join(kept))
                handle.write("\n")


def _from_eval(evaluation: DecisionGateEvaluation) -> DecisionGateRecord:
    return DecisionGateRecord(
        timestamp=evaluation.timestamp.astimezone(timezone.utc).isoformat(),
        symbol=evaluation.symbol,
        timeframe=evaluation.timeframe,
        snapshot_classification=evaluation.snapshot_classification,
        structural_confidence_score=evaluation.structural_confidence_score,
        fast_pressure_score=evaluation.fast_pressure_score,
        fast_structural_alignment=evaluation.fast_structural_alignment,
        fast_structural_divergence_score=evaluation.fast_structural_divergence_score,
        horizon_conflict=evaluation.horizon_conflict,
        cross_horizon_alignment=evaluation.cross_horizon_alignment,
        decision=evaluation.decision,
        reason=evaluation.reason,
        thresholds=dict(evaluation.thresholds),
        mode=evaluation.mode,
    )


def _to_json(item: DecisionGateRecord) -> dict[str, Any]:
    return {
        "timestamp": item.timestamp,
        "symbol": item.symbol,
        "timeframe": item.timeframe,
        "snapshot_classification": item.snapshot_classification,
        "structural_confidence_score": item.structural_confidence_score,
        "fast_pressure_score": item.fast_pressure_score,
        "fast_structural_alignment": item.fast_structural_alignment,
        "fast_structural_divergence_score": item.fast_structural_divergence_score,
        "horizon_conflict": item.horizon_conflict,
        "cross_horizon_alignment": item.cross_horizon_alignment,
        "decision": item.decision,
        "reason": item.reason,
        "thresholds": item.thresholds,
        "mode": item.mode,
    }


def _from_json(payload: dict[str, Any]) -> DecisionGateRecord:
    return DecisionGateRecord(
        timestamp=str(payload.get("timestamp") or datetime.now(timezone.utc).isoformat()),
        symbol=str(payload.get("symbol", "")),
        timeframe=str(payload.get("timeframe")) if payload.get("timeframe") is not None else None,
        snapshot_classification=str(payload.get("snapshot_classification", "non_operable")),
        structural_confidence_score=float(payload["structural_confidence_score"]) if isinstance(payload.get("structural_confidence_score"), (int, float)) else None,
        fast_pressure_score=float(payload["fast_pressure_score"]) if isinstance(payload.get("fast_pressure_score"), (int, float)) else None,
        fast_structural_alignment=str(payload.get("fast_structural_alignment")) if payload.get("fast_structural_alignment") is not None else None,
        fast_structural_divergence_score=float(payload["fast_structural_divergence_score"]) if isinstance(payload.get("fast_structural_divergence_score"), (int, float)) else None,
        horizon_conflict=bool(payload.get("horizon_conflict")) if payload.get("horizon_conflict") is not None else None,
        cross_horizon_alignment=bool(payload.get("cross_horizon_alignment")) if payload.get("cross_horizon_alignment") is not None else None,
        decision=str(payload.get("decision", "bypassed")),
        reason=str(payload.get("reason", "")),
        thresholds=dict(payload.get("thresholds") or {}),
        mode=str(payload.get("mode", "paper_supervised")),
    )
