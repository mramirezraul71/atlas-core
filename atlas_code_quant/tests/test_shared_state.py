from __future__ import annotations

import json
import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.brain.shared_state import SharedStateStore


def test_shared_state_atomic_read_write(tmp_path: Path) -> None:
    p = tmp_path / "state.json"
    ss = SharedStateStore(p)
    out = ss.write_state(lambda d: {**d, "x": 1})
    assert out["x"] == 1
    assert json.loads(p.read_text(encoding="utf-8"))["x"] == 1


def test_shared_state_ensure_defaults(tmp_path: Path) -> None:
    p = tmp_path / "state.json"
    p.write_text("{}", encoding="utf-8")
    ss = SharedStateStore(p)
    out = ss.ensure_defaults({"a": 10, "b": "ok"})
    assert out["a"] == 10
    assert out["b"] == "ok"


def test_shared_state_recovers_invalid_json(tmp_path: Path) -> None:
    p = tmp_path / "state.json"
    p.write_text("{invalid", encoding="utf-8")
    ss = SharedStateStore(p)
    out = ss.read_state()
    assert out == {}
    # Archivo corrupto movido y escritura posterior viable
    ss.write_state(lambda d: {**d, "ok": True})
    assert json.loads(p.read_text(encoding="utf-8"))["ok"] is True

