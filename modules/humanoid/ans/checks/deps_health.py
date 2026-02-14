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
        ok = len(missing) == 0 or all(m in ("ollama running",) for m in missing)
        suggested_heals = []
        message = f"missing={missing}" if missing else "ok"
        details = {"missing_deps": missing}
        if "tesseract" in missing and not _tesseract_binary_exists():
            suggested_heals = ["install_tesseract"]
            message = "Falta binario Tesseract OCR en el OS (no es dependencia pip)"
            details["human_intervention_required"] = "Intervención Humana Requerida: Instalar Tesseract OCR en el OS si winget falla."
        elif "tesseract" in missing and _tesseract_binary_exists():
            missing = [m for m in missing if m != "tesseract"]
            details["missing_deps"] = missing
            message = f"missing={missing}" if missing else "ok"
            ok = len(missing) == 0 or all(m in ("ollama running",) for m in missing)
        elif missing:
            pip_installable = [m for m in missing if m not in ("ollama running",)]
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
    except Exception as e:
        return {"ok": False, "check_id": "deps_health", "message": str(e), "details": {"error": str(e)}, "severity": "low", "suggested_heals": []}
