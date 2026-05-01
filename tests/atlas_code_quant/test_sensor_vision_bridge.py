from __future__ import annotations

import sys
from pathlib import Path

import pytest

# operation_center importa ``backtesting.*`` como paquete de nivel superior
# (árbol bajo atlas_code_quant/). Hay que anteponer ese directorio a sys.path.
ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

pytest.importorskip("pytz", reason="operation_center requiere pytz (requirements-dev)")

from api.schemas import OrderRequest
from operations.operation_center import OperationCenter
from operations.sensor_vision import SensorVisionService


class DummyTracker:
    def build_summary(self, account_scope: str = "paper") -> dict:
        return {
            "pdt_status": {},
            "balances": {},
            "alerts": [],
            "totals": {},
            "account_session": {"scope": account_scope, "account_id": f"{account_scope}-test"},
            "strategies": [],
        }


class DummyJournal:
    def snapshot(self, limit: int = 3) -> dict:
        return {"recent_entries_count": 0, "recent_entries": []}


class DummyExecutor:
    def status(self) -> dict:
        return {"mode": "paper_api", "kill_switch_active": False}

    def configure(self, **kwargs) -> None:
        return None

    def emergency_stop(self, reason: str = "manual_stop") -> None:
        return None

    def clear_emergency_stop(self) -> None:
        return None

    def execute(self, **kwargs) -> dict:
        return {"decision": "noop"}


def test_atlas_push_bridge_status_marks_provider_ready(tmp_path, monkeypatch) -> None:
    service = SensorVisionService(state_path=tmp_path / "vision_state.json")
    service.snapshots_dir = tmp_path / "snapshots"
    service.snapshots_dir.mkdir(parents=True, exist_ok=True)

    monkeypatch.setattr(
        service,
        "_atlas_push_bridge_status",
        lambda: {
            "ok": True,
            "provider": "atlas_push_bridge",
            "bridge_endpoints": {"status": "/api/trading/quant/vision-bridge/status"},
        },
    )

    service.update(provider="atlas_push_bridge")
    status = service.status()

    assert status["provider"] == "atlas_push_bridge"
    assert "atlas_push_bridge" in status["supported_modes"]
    assert status["provider_ready"] is True
    assert status["atlas_push_bridge_available"] is True


def test_atlas_push_bridge_capture_records_snapshot(tmp_path, monkeypatch) -> None:
    service = SensorVisionService(state_path=tmp_path / "vision_state.json")
    service.snapshots_dir = tmp_path / "snapshots"
    service.snapshots_dir.mkdir(parents=True, exist_ok=True)

    monkeypatch.setattr(
        service,
        "_atlas_push_bridge_capture",
        lambda **kwargs: {
            "provider": "atlas_push_bridge",
            "capture_ok": True,
            "capture_path": "C:/snapshots/quant.png",
            "resource_id": "file:C:/snapshots/quant.png",
            "bridge_status": {"ok": True},
        },
    )

    service.update(provider="atlas_push_bridge")
    snapshot = service.capture_context_snapshot(label="quant")

    assert snapshot["provider"] == "atlas_push_bridge"
    assert snapshot["capture_ok"] is True
    assert snapshot["capture_path"] == "C:/snapshots/quant.png"
    assert snapshot["resource_id"] == "file:C:/snapshots/quant.png"
    assert Path(snapshot["meta_path"]).exists()


def test_operation_center_includes_bridge_snapshot_in_cycle(tmp_path, monkeypatch) -> None:
    vision = SensorVisionService(state_path=tmp_path / "vision_state.json")
    vision.snapshots_dir = tmp_path / "snapshots"
    vision.snapshots_dir.mkdir(parents=True, exist_ok=True)

    monkeypatch.setattr(
        vision,
        "_atlas_push_bridge_status",
        lambda: {"ok": True, "provider": "atlas_push_bridge"},
    )
    monkeypatch.setattr(
        vision,
        "_atlas_push_bridge_capture",
        lambda **kwargs: {
            "provider": "atlas_push_bridge",
            "capture_ok": True,
            "capture_path": "C:/snapshots/paper_eval.png",
            "resource_id": "file:C:/snapshots/paper_eval.png",
            "bridge_status": {"ok": True},
        },
    )
    vision.update(provider="atlas_push_bridge")

    center = OperationCenter(
        tracker=DummyTracker(),
        journal=DummyJournal(),
        vision=vision,
        executor=DummyExecutor(),
        state_path=tmp_path / "operation_center_state.json",
    )

    order = OrderRequest(symbol="SPY", side="buy", size=1, order_type="market", account_scope="paper")
    payload = center.evaluate_candidate(order=order, action="evaluate", capture_context=True)

    assert payload["allowed"] is True
    assert payload["vision_snapshot"]["provider"] == "atlas_push_bridge"
    assert payload["vision_snapshot"]["capture_ok"] is True
    assert payload["operation_status"]["vision"]["provider"] == "atlas_push_bridge"


def test_bridge_capture_uses_default_center_target_from_calibration(tmp_path, monkeypatch) -> None:
    service = SensorVisionService(state_path=tmp_path / "vision_state.json")
    service.snapshots_dir = tmp_path / "snapshots"
    service.snapshots_dir.mkdir(parents=True, exist_ok=True)

    calib_path = tmp_path / "screen_gaze_calibration.json"
    calib_path.write_text(
        '{"W": 1920, "H": 1080, "a_y": 2.0, "b_y": -1.0, "a_p": 1.0, "b_p": -0.5, "zoom_center": 1.15}',
        encoding="utf-8",
    )
    monkeypatch.setenv("ATLAS_SCREEN_GAZE_CALIB_PATH", str(calib_path))

    seen: dict[str, object] = {}

    def _capture(**kwargs):
        seen.update(kwargs)
        return {
            "provider": "atlas_push_bridge",
            "capture_ok": True,
            "capture_path": "C:/snapshots/calibrated.png",
            "resource_id": "file:C:/snapshots/calibrated.png",
            "bridge_status": {"ok": True},
        }

    monkeypatch.setattr(service, "_atlas_push_bridge_capture", _capture)
    service.update(provider="atlas_push_bridge")

    snapshot = service.capture_context_snapshot(label="calibration")

    assert snapshot["capture_ok"] is True
    assert seen["calib_path"] == str(calib_path)
    assert seen["screen_target"] == {"x": 960.0, "y": 540.0, "zoom": 1.15}
