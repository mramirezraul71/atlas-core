from __future__ import annotations

from atlas_scanner.contracts import RadarDecisionHandoff, RadarSignalBatch, RadarTimeframe


def build_radar_handoff(batch: RadarSignalBatch) -> RadarDecisionHandoff:
    primary_signal = batch.primary_signal
    primary_timeframe: RadarTimeframe | None = primary_signal.timeframe if primary_signal is not None else None
    primary_scenarios = (
        batch.scenarios_by_timeframe.get(primary_timeframe, ())
        if primary_timeframe is not None
        else ()
    )
    degradation_reasons = tuple(
        reason
        for signal in batch.signals
        for reason in signal.quality.degradation_reasons
    )
    operable = primary_signal.quality.is_operable if primary_signal is not None else False
    summary = (
        f"{batch.symbol} {primary_timeframe} conviction={primary_signal.aggregate_conviction_score:.2f}"
        if primary_signal is not None and primary_timeframe is not None
        else f"{batch.symbol} no_operable_signal"
    )
    return RadarDecisionHandoff(
        symbol=batch.symbol,
        as_of=batch.as_of,
        operable=operable,
        primary_timeframe=primary_timeframe,
        primary_signal=primary_signal,
        primary_scenarios=tuple(primary_scenarios),
        signals=batch.signals,
        degradation_reasons=degradation_reasons,
        handoff_summary=summary,
        metadata={
            "source": "atlas_scanner.inference.handoff",
            "timeframes_evaluated": tuple(signal.timeframe for signal in batch.signals),
            "snapshot_classification": (
                primary_signal.meta.get("snapshot_classification")
                if primary_signal is not None and isinstance(primary_signal.meta, dict)
                else "non_operable"
            ),
        },
    )
