from __future__ import annotations

from atlas_scanner.contracts import RadarSignal
from atlas_scanner.fusion import RadarFeatureVector


def aggregate_conviction(feature_vector: RadarFeatureVector) -> float:
    intraday = feature_vector.horizon_scores.get("intraday", 50.0)
    swing = feature_vector.horizon_scores.get("swing", 50.0)
    positional = feature_vector.horizon_scores.get("positional", 50.0)
    return (intraday * 0.45) + (swing * 0.35) + (positional * 0.20)


def build_primary_reason(feature_vector: RadarFeatureVector) -> str:
    top_name, top_score = max(
        (
            ("direction", feature_vector.direction),
            ("volume_confirmation", feature_vector.volume_confirmation),
            ("flow_conviction", feature_vector.flow_conviction),
            ("dte_pressure", feature_vector.dte_pressure),
            ("dealer_proxy", feature_vector.dealer_proxy),
        ),
        key=lambda item: item[1],
    )
    return f"{top_name}:{top_score:.2f}"


def build_degradation_reason(signal: RadarSignal) -> str | None:
    if not signal.quality.is_degraded:
        return None
    if signal.quality.degradation_reasons:
        return signal.quality.degradation_reasons[0]
    return "degraded_without_reason"
