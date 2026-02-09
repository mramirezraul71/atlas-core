# -*- coding: utf-8 -*-
from datetime import datetime
from pathlib import Path

LOG_FILE = Path(r"C:\ATLAS\logs\atlas.log")

def log(message: str):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    entry = f"[{ts}] {message}\n"
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(LOG_FILE, "a", encoding="utf-8", errors="replace") as f:
        f.write(entry)
