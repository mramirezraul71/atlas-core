from __future__ import annotations

import json
import sys
from pathlib import Path

QUANT_ROOT = Path(__file__).resolve().parents[1]
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_core.adapters.quant_adapter import QuantBrainAdapter
from atlas_core.brain.models import Command


def test_quant_setdefault_and_reduce_risk(tmp_path: Path) -> None:
    p = tmp_path / "operation_center_state.json"
    p.write_text("{}", encoding="utf-8")
    ad = QuantBrainAdapter(operation_state_path=p)
    ad.get_state()
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data.get("autonomy_mode") == "semi"
    assert data.get("max_risk_per_trade_pct") == 0.02

    ad.apply_command(Command(target="quant", action="reduce_risk", params={"factor": 0.5}))
    data2 = json.loads(p.read_text(encoding="utf-8"))
    assert data2["max_risk_per_trade_pct"] < 0.02


def test_quant_pause_trading(tmp_path: Path) -> None:
    p = tmp_path / "operation_center_state.json"
    p.write_text(json.dumps({"auton_mode": "paper_autonomous"}), encoding="utf-8")
    ad = QuantBrainAdapter(operation_state_path=p)
    out = ad.apply_command(Command(target="quant", action="pause_trading", params={}))
    assert out.get("ok") is True
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data.get("auton_mode") == "off"
    assert data.get("fail_safe_active") is True


def test_quant_set_mode(tmp_path: Path) -> None:
    p = tmp_path / "operation_center_state.json"
    p.write_text("{}", encoding="utf-8")
    ad = QuantBrainAdapter(operation_state_path=p)
    ad.apply_command(Command(target="quant", action="set_mode", params={"mode": "safe"}))
    data = json.loads(p.read_text(encoding="utf-8"))
    assert data.get("autonomy_mode") == "safe"


def test_quant_adapter_recovers_corrupt_json(tmp_path: Path) -> None:
    p = tmp_path / "operation_center_state.json"
    p.write_text("{bad-json", encoding="utf-8")
    ad = QuantBrainAdapter(operation_state_path=p)
    st = ad.get_state()
    assert st.name == "quant"
    # Debe poder reescribir luego de recuperar corrupción
    out = ad.apply_command(Command(target="quant", action="set_mode", params={"mode": "semi"}))
    assert out.get("ok") is True


def test_quant_default_path_prefers_canonical_quant_state(monkeypatch, tmp_path: Path) -> None:
    ad = QuantBrainAdapter()
    expected = Path(__file__).resolve().parents[2] / "data" / "operation" / "operation_center_state.json"
    assert ad._path == expected
