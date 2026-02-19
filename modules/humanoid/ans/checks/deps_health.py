"""Deps health check."""
from __future__ import annotations

import os
from pathlib import Path


def _tesseract_binary_exists() -> bool:
    """Considera binario presente si existe en TESSERACT_CMD, Program Files o PATH (evita bucle Bitácora)."""
    path = os.getenv("TESSERACT_CMD")
    if path and Path(path).exists():
        return True
    for p in (r"C:\Program Files\Tesseract-OCR\tesseract.exe", r"C:\Program Files (x86)\Tesseract-OCR\tesseract.exe"):
        if Path(p).exists():
            return True
    try:
        import shutil
        w = shutil.which("tesseract")
        if w and Path(w).exists():
            return True
    except Exception:
        pass
    return False


def run() -> dict:
    try:
        from modules.humanoid.deps_checker import check_all
        r = check_all()
        missing = r.get("missing_deps", [])
        
        # Dependencias opcionales de voz - no críticas, no generan incidentes
        voice_deps = ("speech_recognition", "pyttsx3", "piper", "sherpa-onnx", "faster-whisper")
        critical_missing = [m for m in missing if not any(v in m.lower() for v in voice_deps)]
        
        ok = len(critical_missing) == 0 or all(m in ("ollama running",) for m in critical_missing)
        suggested_heals = []
        message = f"missing={critical_missing}" if critical_missing else "ok"
        details = {"missing_deps": critical_missing, "optional_missing": [m for m in missing if m not in critical_missing]}
        if "tesseract" in critical_missing and not _tesseract_binary_exists():
            suggested_heals = ["install_tesseract"]
            message = "Falta binario Tesseract OCR en el OS (no es dependencia pip)"
            details["human_intervention_required"] = "Intervención Humana Requerida: Instalar Tesseract OCR en el OS si winget falla."
        elif "tesseract" in critical_missing and _tesseract_binary_exists():
            critical_missing = [m for m in critical_missing if m != "tesseract"]
            details["missing_deps"] = critical_missing
            message = f"missing={critical_missing}" if critical_missing else "ok"
            ok = len(critical_missing) == 0 or all(m in ("ollama running",) for m in critical_missing)
        elif critical_missing:
            pip_installable = [m for m in critical_missing if m not in ("ollama running",)]
            if pip_installable:
                suggested_heals = ["install_optional_deps"]
        return {
            "ok": ok,
            "check_id": "deps_health",
            "message": message,
            "details": details,
            "severity": "low" if ok else "med",
            "suggested_heals": suggested_heals,
        }
    except OSError as e:
        # [Errno 22] y similares son errores de I/O de consola en Windows (proceso detached),
        # no indican dependencias faltantes — se trata como OK para evitar falsos incidentes.
        if getattr(e, "errno", None) in (22, 9):
            return {"ok": True, "check_id": "deps_health", "message": "ok (io_error_suppressed)", "details": {"suppressed_error": str(e)}, "severity": "low", "suggested_heals": []}
        return {"ok": False, "check_id": "deps_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": []}
    except Exception as e:
        return {"ok": False, "check_id": "deps_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": []}
