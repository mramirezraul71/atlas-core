import os
import re
import json
from datetime import datetime
from pathlib import Path

# =========================
# Paths / Config
# =========================
ATLAS_ROOT = Path(os.getenv("ATLAS_ROOT", r"C:\ATLAS"))
VAULT_DIR  = ATLAS_ROOT / "ATLAS_VAULT"
NOTES_DIR  = VAULT_DIR / "NOTES"
LOGS_DIR   = ATLAS_ROOT / "logs"
SNAPS_DIR  = ATLAS_ROOT / "snapshots"
LOG_FILE   = LOGS_DIR / "atlas.log"

def _ensure_dirs():
    NOTES_DIR.mkdir(parents=True, exist_ok=True)
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    SNAPS_DIR.mkdir(parents=True, exist_ok=True)

def _now():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def _log(msg: str):
    _ensure_dirs()
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{_now()}] {msg}\n")

# =========================
# CORE
# =========================
def status() -> str:
    _ensure_dirs()
    return f"ATLAS OK | logs={LOG_FILE} | snapshots={SNAPS_DIR}"

def doctor() -> str:
    try:
        _ensure_dirs()
        _log("doctor OK")
        return "ATLAS DOCTOR: OK"
    except Exception as e:
        _log(f"doctor FAIL {e}")
        return f"ATLAS DOCTOR FAIL: {e}"

def modules_report() -> str:
    return (
        "Módulos activos:\n"
        "- Notes Vault\n"
        "- Logs\n"
        "- Snapshots\n"
        "- Doctor\n"
        "- Runtime Router\n"
    )

def handle(text: str) -> str:
    if not text:
        return "ATLAS vacío."

    t = text.lower().strip()

    if t.startswith("/status"):
        return status()
    if t.startswith("/doctor"):
        return doctor()
    if t.startswith("/modules"):
        return modules_report()

    _log(f"inbox: {text}")
    return f"ATLAS recibió: {text}"
