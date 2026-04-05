from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

from config import VisionSensorConfig
from multimodal_fusion import MultimodalFusion


class _DummyLogger:
    def info(self, *args, **kwargs):  # pragma: no cover
        return None

    def warning(self, *args, **kwargs):  # pragma: no cover
        return None


def test_multimodal_fusion_degrades_on_contradictions(tmp_path: Path) -> None:
    config = VisionSensorConfig.load()
    config = replace(config, quant_state_path=tmp_path / "operation_center_state.json")
    config.quant_state_path.write_text(
        json.dumps({"last_operational_error": {"reason": "reconciliation_failed"}, "last_decision": {"allowed": False}}),
        encoding="utf-8",
    )
    fusion = MultimodalFusion(config, _DummyLogger())
    telemetry = fusion.snapshot()
    decision = fusion.fuse(
        visual_result={"pattern": "uptrend", "confidence": 0.72, "payload": {"chart_bias": "bullish"}},
        candle_result={"confidence": 0.7, "payload": {"top_direction": "bearish"}},
        geometry_result={"confidence": 0.4, "hits": [{"pattern": "channel"}], "payload": {}},
        telemetry_snapshot=telemetry,
    )
    assert decision.action == "degrade"
    assert decision.contradictions
