from __future__ import annotations

import base64
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _logs_path() -> Path:
    raw = (os.getenv("ATLAS_WORLD_STATE_PATH") or "").strip()
    if raw:
        return Path(raw)
    return _repo_root() / "logs" / "world_state.json"


def _snapshots_dir() -> Path:
    return _repo_root() / "snapshots" / "vision" / "world_state"


def _safe_write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def _quality_metrics(image_path: str) -> Dict[str, Any]:
    """
    Métricas simples y rápidas (offline, sin OpenCV obligatorio):
    - brightness_mean: 0..255
    - contrast_std: desviación estándar de luminancia
    - edge_mean: heurística de nitidez (FIND_EDGES) 0..255
    """
    try:
        from PIL import Image, ImageFilter, ImageStat
    except Exception:
        return {"ok": False, "error": "pillow_not_available"}
    try:
        p = Path(image_path)
        img = Image.open(p).convert("RGB")
        gray = img.convert("L")
        stat = ImageStat.Stat(gray)
        edges = gray.filter(ImageFilter.FIND_EDGES)
        estat = ImageStat.Stat(edges)
        return {
            "ok": True,
            "width": img.size[0],
            "height": img.size[1],
            "brightness_mean": float(stat.mean[0]) if stat.mean else 0.0,
            "contrast_std": float(stat.stddev[0]) if stat.stddev else 0.0,
            "edge_mean": float(estat.mean[0]) if estat.mean else 0.0,
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


@dataclass
class WorldState:
    ts: float
    source: str
    eye: str = ""
    cam_id: str = ""
    image_path: str = ""
    ocr_text: str = ""
    ocr_error: str = ""
    ocr_items: list[dict] = field(default_factory=list)
    vision_interpretation: str = ""
    vision_error: str = ""
    quality: dict = field(default_factory=dict)
    ok: bool = True
    error: str = ""


def capture_world_state(*, eye: Optional[str] = None, include_ocr_items: bool = False, use_llm_vision: bool = False) -> Dict[str, Any]:
    """
    Captura un "estado del mundo" minimalista y persistible.
    - Captura imagen (ubiq/nexus/local)
    - Calcula métricas de calidad
    - Ejecuta OCR (texto + opcional bounding boxes)
    - (opcional) LLM vision para interpretación
    """
    from modules.humanoid.nerve.eyes import eyes_capture

    snap = eyes_capture(use_nexus_if_available=True, source="camera", enhance="auto", eye=eye)
    if not snap.get("ok") or not snap.get("image_base64"):
        ws = WorldState(ts=time.time(), source=str(snap.get("source") or "unknown"), ok=False, error=str(snap.get("error") or "capture_failed"))
        out = asdict(ws)
        try:
            _safe_write_json(_logs_path(), out)
        except Exception:
            pass
        return out

    b64 = str(snap.get("image_base64") or "")
    img_bytes = b""
    try:
        img_bytes = base64.b64decode(b64)
    except Exception:
        img_bytes = b""

    # Persistir evidencia (último frame)
    sdir = _snapshots_dir()
    sdir.mkdir(parents=True, exist_ok=True)
    img_path = sdir / "latest.jpg"
    try:
        if img_bytes:
            img_path.write_bytes(img_bytes)
    except Exception:
        pass

    # OCR
    ocr_text = ""
    ocr_err = ""
    ocr_items: list[dict] = []
    try:
        from modules.humanoid.screen.ocr import run_ocr, run_ocr_data

        ocr_text, ocr_err = run_ocr(image_path=str(img_path))
        if include_ocr_items:
            ocr_items, _ = run_ocr_data(image_path=str(img_path))
    except Exception as e:
        ocr_err = str(e)

    # Interpretación (opcional)
    interp = ""
    v_err = ""
    if use_llm_vision:
        try:
            from modules.humanoid.vision.analyzer import analyze_with_llm

            r = analyze_with_llm(str(img_path), prompt="Describe la escena en una frase. Si hay texto, resume lo esencial.")
            if r.get("ok"):
                interp = (r.get("interpretation") or "").strip()
            else:
                v_err = (r.get("error") or "llm_vision_failed")
        except Exception as e:
            v_err = str(e)

    q = _quality_metrics(str(img_path))
    ws = WorldState(
        ts=time.time(),
        source=str(snap.get("source") or ""),
        eye=str(snap.get("eye") or ""),
        cam_id=str(snap.get("cam_id") or ""),
        image_path=str(img_path),
        ocr_text=ocr_text,
        ocr_error=ocr_err,
        ocr_items=ocr_items if include_ocr_items else [],
        vision_interpretation=interp,
        vision_error=v_err,
        quality=q,
        ok=True,
        error="",
    )
    out = asdict(ws)
    try:
        _safe_write_json(_logs_path(), out)
    except Exception:
        pass
    return out


def load_latest_world_state() -> Dict[str, Any]:
    p = _logs_path()
    try:
        if not p.is_file():
            return {"ok": False, "error": "no_world_state_yet"}
        return json.loads(p.read_text(encoding="utf-8", errors="replace"))
    except Exception as e:
        return {"ok": False, "error": str(e)}

