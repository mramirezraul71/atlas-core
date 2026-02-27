from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict


def atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    """PY005: escritura atómica Windows-safe de JSON UTF-8.

    Reglas esperadas (ver tests):
    - escribe en tmp en el mismo directorio
    - reemplaza con os.replace (atómico)
    - limpia tmp si algo falla
    """
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)

    tmp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=str(p.parent),
            delete=False,
            suffix=".tmp",
        ) as f:
            tmp_name = f.name
            f.write(content)
        os.replace(tmp_name, str(p))
    finally:
        try:
            if tmp_name and Path(tmp_name).exists():
                Path(tmp_name).unlink(missing_ok=True)
        except Exception:
            pass

