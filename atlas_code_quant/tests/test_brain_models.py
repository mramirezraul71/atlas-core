from __future__ import annotations

import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.models import (
    Command,
    Event,
    ModuleState,
    RiskState,
    SystemSnapshot,
)


def test_models_fields() -> None:
    ev = Event(source="op", kind="k", payload={"a": 1})
    assert ev.source == "op" and ev.kind == "k" and ev.payload["a"] == 1

    cmd = Command(target="body", action="speak", params={"text": "hola"})
    assert cmd.target == "body" and cmd.action == "speak"

    ms = ModuleState(name="vision", health="degraded", mode="on", details={"x": 2})
    assert ms.health == "degraded"

    rs = RiskState(name="quant", level="high", details={})
    assert rs.level == "high"

    snap = SystemSnapshot(
        global_mode="semi",
        active_mission=None,
        modules={},
        risks={},
        meta={},
    )
    assert snap.global_mode == "semi"
