from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from atlas_scanner.contracts import InterpretedScenario, RadarSignal, RadarSignalBatch


@dataclass(frozen=True)
class SnapshotRecord:
    symbol: str
    timeframe: str
    timestamp: str
    operable: bool
    aggregate_conviction_score: float
    signal: dict[str, Any]
    scenarios: tuple[dict[str, Any], ...]
    quality_flags: dict[str, Any]
    provider_diagnostics: dict[str, Any]
    degradation_reasons: tuple[str, ...]


class JsonlRadarSnapshotStore:
    def __init__(
        self,
        path: Path,
        *,
        max_records: int = 5000,
        max_bytes: int = 8_000_000,
    ) -> None:
        self.path = path
        self.max_records = max(100, int(max_records))
        self.max_bytes = max(100_000, int(max_bytes))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.path.write_text("", encoding="utf-8")

    def append_batch(
        self,
        *,
        batch: RadarSignalBatch,
        provider_health: dict[str, Any] | None = None,
    ) -> tuple[SnapshotRecord, ...]:
        self._rotate_if_needed(incoming_records=max(1, len(batch.signals)))
        records: list[SnapshotRecord] = []
        emitted_at = datetime.now(timezone.utc).isoformat()
        for signal in batch.signals:
            scenarios = batch.scenarios_by_timeframe.get(signal.timeframe, ())
            record = SnapshotRecord(
                symbol=batch.symbol,
                timeframe=signal.timeframe,
                timestamp=emitted_at,
                operable=signal.quality.is_operable,
                aggregate_conviction_score=signal.aggregate_conviction_score,
                signal=_signal_to_dict(signal),
                scenarios=tuple(_scenario_to_dict(item) for item in scenarios),
                quality_flags=_quality_to_dict(signal),
                provider_diagnostics=dict(provider_health or {}),
                degradation_reasons=signal.quality.degradation_reasons,
            )
            records.append(record)
        with self.path.open("a", encoding="utf-8") as handle:
            for item in records:
                handle.write(json.dumps(_record_to_json(item), ensure_ascii=False))
                handle.write("\n")
        return tuple(records)

    def stats(self) -> dict[str, Any]:
        if not self.path.exists():
            return {
                "path": str(self.path),
                "total_records": 0,
                "file_size_bytes": 0,
                "oldest_timestamp": None,
                "newest_timestamp": None,
            }
        lines = self.path.read_text(encoding="utf-8").splitlines()
        timestamps: list[str] = []
        for raw in lines:
            if not raw.strip():
                continue
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                continue
            ts = payload.get("timestamp")
            if isinstance(ts, str) and ts:
                timestamps.append(ts)
        timestamps_sorted = sorted(timestamps)
        return {
            "path": str(self.path),
            "total_records": len(lines),
            "file_size_bytes": self.path.stat().st_size,
            "oldest_timestamp": timestamps_sorted[0] if timestamps_sorted else None,
            "newest_timestamp": timestamps_sorted[-1] if timestamps_sorted else None,
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
        # Keep only the newest half to avoid uncontrolled growth.
        keep_from = max(0, len(lines) - (self.max_records // 2))
        kept = lines[keep_from:]
        with self.path.open("w", encoding="utf-8") as handle:
            if kept:
                handle.write("\n".join(kept))
                handle.write("\n")

    def recent(
        self,
        *,
        limit: int = 50,
        symbol: str | None = None,
        timeframe: str | None = None,
        only_operable: bool = False,
        include_degraded: bool = True,
    ) -> tuple[SnapshotRecord, ...]:
        lines = self.path.read_text(encoding="utf-8").splitlines()
        out: list[SnapshotRecord] = []
        for raw in reversed(lines):
            if not raw.strip():
                continue
            payload = json.loads(raw)
            record = _record_from_json(payload)
            if symbol and record.symbol.upper() != symbol.upper():
                continue
            if timeframe and record.timeframe != timeframe:
                continue
            if only_operable and not record.operable:
                continue
            if not include_degraded and record.degradation_reasons:
                continue
            out.append(record)
            if len(out) >= max(1, limit):
                break
        return tuple(out)

    def replay(
        self,
        *,
        symbol: str,
        timeframe: str | None = None,
        limit: int = 100,
        include_degraded: bool = True,
    ) -> tuple[SnapshotRecord, ...]:
        return self.recent(
            limit=limit,
            symbol=symbol,
            timeframe=timeframe,
            only_operable=False,
            include_degraded=include_degraded,
        )


def _signal_to_dict(signal: RadarSignal) -> dict[str, Any]:
    return {
        "symbol": signal.symbol,
        "timeframe": signal.timeframe,
        "as_of": signal.as_of.isoformat(),
        "direction_score": signal.direction_score,
        "volume_confirmation_score": signal.volume_confirmation_score,
        "flow_conviction_score": signal.flow_conviction_score,
        "dte_pressure_score": signal.dte_pressure_score,
        "dealer_positioning_proxy_score": signal.dealer_positioning_proxy_score,
        "aggregate_conviction_score": signal.aggregate_conviction_score,
        "primary_conviction_reason": signal.primary_conviction_reason,
        "primary_degradation_reason": signal.primary_degradation_reason,
        "meta": dict(signal.meta),
    }


def _scenario_to_dict(item: InterpretedScenario) -> dict[str, Any]:
    return {
        "name": item.name,
        "conviction": item.conviction,
        "probability": item.probability,
        "explanation": item.explanation,
        "misinterpretation_risks": list(item.misinterpretation_risks),
        "invalidation_conditions": list(item.invalidation_conditions),
        "ambiguity_flags": list(item.ambiguity_flags),
        "meta": dict(item.meta),
    }


def _quality_to_dict(signal: RadarSignal) -> dict[str, Any]:
    return {
        "has_market_data": signal.quality.has_market_data,
        "has_flow_data": signal.quality.has_flow_data,
        "has_greeks_context": signal.quality.has_greeks_context,
        "has_oi_context": signal.quality.has_oi_context,
        "provider_quality_ok": signal.quality.provider_quality_ok,
        "is_degraded": signal.quality.is_degraded,
        "is_operable": signal.quality.is_operable,
        "degradation_reasons": list(signal.quality.degradation_reasons),
    }


def _record_to_json(item: SnapshotRecord) -> dict[str, Any]:
    return {
        "symbol": item.symbol,
        "timeframe": item.timeframe,
        "timestamp": item.timestamp,
        "operable": item.operable,
        "aggregate_conviction_score": item.aggregate_conviction_score,
        "signal": item.signal,
        "scenarios": list(item.scenarios),
        "quality_flags": item.quality_flags,
        "provider_diagnostics": item.provider_diagnostics,
        "degradation_reasons": list(item.degradation_reasons),
    }


def _record_from_json(payload: dict[str, Any]) -> SnapshotRecord:
    return SnapshotRecord(
        symbol=str(payload.get("symbol", "")),
        timeframe=str(payload.get("timeframe", "")),
        timestamp=str(payload.get("timestamp", "")),
        operable=bool(payload.get("operable", False)),
        aggregate_conviction_score=float(payload.get("aggregate_conviction_score", 0.0)),
        signal=dict(payload.get("signal") or {}),
        scenarios=tuple(payload.get("scenarios") or ()),
        quality_flags=dict(payload.get("quality_flags") or {}),
        provider_diagnostics=dict(payload.get("provider_diagnostics") or {}),
        degradation_reasons=tuple(payload.get("degradation_reasons") or ()),
    )
