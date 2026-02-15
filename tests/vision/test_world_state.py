from __future__ import annotations

import base64
from pathlib import Path


def _make_jpeg_bytes() -> bytes:
    from PIL import Image
    import io

    img = Image.new("RGB", (64, 48), color=(120, 200, 80))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def test_capture_world_state_persists(monkeypatch, tmp_path: Path):
    jpeg = _make_jpeg_bytes()
    b64 = base64.b64encode(jpeg).decode("ascii")

    # Force paths to temp
    monkeypatch.setenv("ATLAS_WORLD_STATE_PATH", str(tmp_path / "world_state.json"))

    def fake_eyes_capture(**kwargs):
        return {"ok": True, "image_base64": b64, "source": "local", "evidence_path": None, "error": None}

    import modules.humanoid.vision.world_state as ws

    monkeypatch.setattr(ws, "_snapshots_dir", lambda: tmp_path / "snapshots")
    monkeypatch.setattr("modules.humanoid.nerve.eyes.eyes_capture", fake_eyes_capture)

    out = ws.capture_world_state(include_ocr_items=False, use_llm_vision=False)
    assert out["ok"] is True
    assert out["source"] in ("local", "nexus", "ubiq")
    assert "quality" in out and out["quality"].get("ok") in (True, False)

    # persisted
    assert (tmp_path / "world_state.json").is_file()
    latest = ws.load_latest_world_state()
    assert isinstance(latest, dict)

