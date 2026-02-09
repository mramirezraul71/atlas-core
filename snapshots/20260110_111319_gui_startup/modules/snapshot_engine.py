# -*- coding: utf-8 -*-
import shutil
from datetime import datetime
from pathlib import Path
from core.logger import log

BASE = Path(r"C:\ATLAS")
SNAPSHOTS = BASE / "snapshots"

def snapshot(label="auto"):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{ts}_{label}"
    dest = SNAPSHOTS / name
    dest.mkdir(parents=True, exist_ok=True)

    # Copiar carpetas clave
    for folder in ["core", "modules", "rauli", "config", "memory", "logs"]:
        src = BASE / folder
        if src.exists():
            shutil.copytree(src, dest / folder, dirs_exist_ok=True)

    # Copiar atlas.py si existe
    atlas = BASE / "atlas.py"
    if atlas.exists():
        shutil.copy2(atlas, dest / "atlas.py")

    log(f"Snapshot creado: {name}")
    return str(dest)
