# -*- coding: utf-8 -*-
import os, subprocess
from datetime import datetime
from typing import Optional, Dict, Any
from fastapi import FastAPI, Header, HTTPException
from pydantic import BaseModel

BASE = r"C:\ATLAS"
LOG_DIR = os.path.join(BASE, "bridge", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, "bridge.log")

# Token del puente (NO es tu OpenAI key). Este token protege tu PC.
ATLAS_TOKEN = os.getenv("ATLAS_TOKEN", "cambia_esto_ya")

def _log(msg: str):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{ts}] {msg}\n")

def _auth(authorization: Optional[str]):
    # Espera: Authorization: Bearer <token>
    if not authorization or not authorization.lower().startswith("bearer "):
        raise HTTPException(status_code=401, detail="Missing Bearer token")
    token = authorization.split(" ", 1)[1].strip()
    if token != ATLAS_TOKEN:
        raise HTTPException(status_code=401, detail="Invalid token")

def _run(cmd: str, timeout: int = 120) -> Dict[str, Any]:
    _log(f"RUN: {cmd}")
    try:
        p = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=BASE
        )
        out = (p.stdout or "")[-8000:]
        err = (p.stderr or "")[-8000:]
        _log(f"EXIT={p.returncode}")
        if out: _log("STDOUT:\n" + out)
        if err: _log("STDERR:\n" + err)
        return {"ok": p.returncode == 0, "code": p.returncode, "stdout": out, "stderr": err}
    except subprocess.TimeoutExpired:
        _log("TIMEOUT")
        return {"ok": False, "code": -1, "stdout": "", "stderr": "TIMEOUT"}

# -------------------------
# MODELOS
# -------------------------
class CommandReq(BaseModel):
    action: str
    args: Optional[Dict[str, Any]] = None
    confirm: bool = True  # ya lo dejamos en true por defecto para fluidez

app = FastAPI(title="ATLAS Bridge", version="2.0.0")

@app.get("/status")
def status(authorization: Optional[str] = Header(None)):
    _auth(authorization)
    return {"ok": True, "time": datetime.now().isoformat(), "base": BASE, "log": LOG_FILE}

@app.get("/logs")
def logs(tail: int = 200, authorization: Optional[str] = Header(None)):
    _auth(authorization)
    if not os.path.exists(LOG_FILE):
        return {"ok": True, "lines": []}
    with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()
    return {"ok": True, "lines": lines[-max(1, min(2000, tail)):]}

@app.post("/command")
def command(req: CommandReq, authorization: Optional[str] = Header(None)):
    _auth(authorization)

    a = req.action.lower().strip()
    args = req.args or {}

    # Acciones PRO (lista blanca)
    if a == "flutter_doctor":
        return _run("flutter doctor -v", timeout=240)

    if a == "run_rauli":
        proj = args.get("path", r"C:\ATLAS\rauli_core_app")
        cmd = f'cd /d "{proj}" && flutter run -d windows'
        return _run(cmd, timeout=600)

    if a == "cmd":
        raw = (args.get("line") or "").strip()
        banned = ["format", "shutdown", "reg delete", "rd /s", "Remove-Item", "del /f", "diskpart"]
        if any(b in raw.lower() for b in banned):
            _log(f"BLOCKED cmd: {raw}")
            return {"ok": False, "code": 403, "stdout": "", "stderr": "BLOCKED"}
        return _run(raw, timeout=int(args.get("timeout", 120)))

    return {"ok": False, "code": 404, "stdout": "", "stderr": f"Unknown action: {req.action}"}
