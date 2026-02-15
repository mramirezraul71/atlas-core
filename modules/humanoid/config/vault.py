from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

_LOADED = False


def _candidate_paths() -> list[str]:
    """
    Bóveda (credenciales.txt). Orden:
    1) ATLAS_VAULT_PATH (si existe)
    2) Ruta owner (Rauli-Vision)
    3) Fallback histórico (C:\\dev\\credenciales.txt)
    """
    out: list[str] = []
    p = (os.getenv("ATLAS_VAULT_PATH") or "").strip()
    if p:
        out.append(p)
    out.append(r"C:\Users\Raul\OneDrive\RAUL - Personal\Escritorio\credenciales.txt")
    out.append(r"C:\dev\credenciales.txt")
    return out


def load_vault_env(*, override: bool = False) -> Optional[str]:
    """
    Carga variables de entorno desde la Bóveda (credenciales.txt).
    Best-effort: si no existe o falla, no rompe.

    Retorna la ruta usada (str) o None.
    """
    global _LOADED
    if _LOADED and not override:
        return None
    try:
        from dotenv import load_dotenv
    except Exception:
        return None

    for p in _candidate_paths():
        try:
            path = Path(p)
            if not path.is_file():
                continue
            # Silenciar warnings/errores de parseo de python-dotenv (no ensuciar consola).
            import io
            import contextlib

            sink = io.StringIO()
            with contextlib.redirect_stderr(sink), contextlib.redirect_stdout(sink):
                load_dotenv(str(path), override=override)
            _LOADED = True
            return str(path)
        except Exception:
            continue
    return None

