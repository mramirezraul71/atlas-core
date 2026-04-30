"""Pistas auxiliares Windows para diagnóstico de cámara en Quant (sin depender del robot :8002).

Solo apoyo: enumeración PnP (Camera/Image) vía PowerShell con timeout corto.
Fallos silenciosos → lista vacía; nunca lanza hacia el caller de health.
"""
from __future__ import annotations

import json
import logging
import subprocess
import sys
from typing import Any

logger = logging.getLogger("atlas.vision.windows_camera_hints")


def fetch_pnp_camera_hints(*, timeout_sec: float = 2.5) -> list[dict[str, Any]]:
    """Devuelve dispositivos PnP OK (nombre + clase). Windows solamente."""
    if sys.platform != "win32":
        return []
    cmd = (
        "$c = Get-PnpDevice -Class Camera -EA SilentlyContinue | Select-Object Status,FriendlyName,Class; "
        "$i = Get-PnpDevice -Class Image -EA SilentlyContinue | Select-Object Status,FriendlyName,Class; "
        "@($c + $i) | ConvertTo-Json -Compress -Depth 3"
    )
    try:
        proc = subprocess.run(
            ["powershell", "-NoProfile", "-Command", cmd],
            capture_output=True,
            text=True,
            timeout=max(0.5, float(timeout_sec)),
            check=False,
        )
    except Exception as exc:
        logger.debug("PnP camera hints omitidos: %s", exc)
        return []
    if proc.returncode != 0 or not (proc.stdout or "").strip():
        return []
    raw = proc.stdout.strip()
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        try:
            data = json.loads("[" + raw.replace("}\n{", "},{") + "]")
        except json.JSONDecodeError:
            logger.debug("PnP hints JSON no parseable")
            return []
    rows = data if isinstance(data, list) else [data]
    out: list[dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        if str(row.get("Status") or "").strip().upper() != "OK":
            continue
        name = str(row.get("FriendlyName") or "").strip()
        if not name:
            continue
        cls = str(row.get("Class") or "").strip()
        out.append({"name": name, "class": cls})
    return out[:24]
