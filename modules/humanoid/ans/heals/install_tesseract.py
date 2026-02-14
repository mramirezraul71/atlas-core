"""Instala el motor binario Tesseract OCR vía winget (Windows). Dependencia de OS, no pip."""
from __future__ import annotations

import sys
import subprocess
from .base import heal_result

_WINGET_ID = "UB-Mannheim.TesseractOCR"
_CONTINGENCY_MSG = (
    "ATLAS REQUIERE INTERVENCIÓN: Descarga e instala el binario oficial Tesseract v5.x (64-bit) desde: "
    "https://github.com/UB-Mannheim/tesseract/wiki/Downloading-Tesseract-OCR. "
    "Usa el instalador 'tesseract-ocr-w64-setup-5.3.3.xxxx.exe'. "
    "Una vez instalado en 'C:\\Program Files\\Tesseract-OCR', presiona Enter para que ATLAS continúe."
)


def run(**kwargs) -> dict:
    if sys.platform != "win32":
        return heal_result(False, "install_tesseract", "Solo Windows: usa tu gestor de paquetes", {})
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("heal_attempt", heal_id="install_tesseract", message=f"winget install -e --id {_WINGET_ID}")
    except Exception:
        pass
    try:
        r = subprocess.run(
            ["winget", "install", "-e", "--id", _WINGET_ID, "--accept-source-agreements", "--accept-package-agreements"],
            capture_output=True,
            timeout=300,
            text=True,
            creationflags=subprocess.CREATE_NO_WINDOW if hasattr(subprocess, "CREATE_NO_WINDOW") else 0,
        )
        out = (r.stdout or "") + (r.stderr or "")
        already_installed = "ya instalado" in out.lower() or "already installed" in out.lower() or "no hay versiones más recientes" in out.lower()
        ok = r.returncode == 0 or already_installed
        if ok:
            msg = "Tesseract instalado (winget)" if r.returncode == 0 else "Tesseract ya estaba instalado"
            try:
                from modules.humanoid.ans.live_stream import emit
                emit("heal_end", heal_id="install_tesseract", ok=True, message=msg)
            except Exception:
                pass
            return heal_result(True, "install_tesseract", msg, {"winget_exit": r.returncode})
        _print_contingency_and_wait()
        return heal_result(False, "install_tesseract", "winget falló; intervención humana requerida", {"winget_exit": r.returncode, "output": out[:500]})
    except FileNotFoundError:
        _print_contingency_and_wait()
        return heal_result(False, "install_tesseract", "winget no encontrado (Windows App Installer)", {})
    except subprocess.TimeoutExpired:
        _print_contingency_and_wait()
        return heal_result(False, "install_tesseract", "winget timeout (300s)", {})
    except Exception as e:
        _print_contingency_and_wait()
        return heal_result(False, "install_tesseract", str(e), {}, str(e))


def _print_contingency_and_wait() -> None:
    print(_CONTINGENCY_MSG)
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
