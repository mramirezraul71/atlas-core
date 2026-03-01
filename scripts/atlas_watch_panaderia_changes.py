from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List


BASE = Path(__file__).resolve().parent.parent
PAN = BASE / "_external" / "rauli-panaderia"
LOG = BASE / "logs" / "atlas_watch_panaderia.log"
LOG.parent.mkdir(parents=True, exist_ok=True)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _log(payload: dict) -> None:
    with LOG.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def _run(cmd: List[str], cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        capture_output=True,
        text=True,
        timeout=30,
    )


def _git_logic_fingerprint() -> str:
    if not (PAN / ".git").exists():
        return "no_repo"
    r = _run(["git", "status", "--porcelain"], cwd=PAN)
    lines: List[str] = []
    for ln in (r.stdout or "").splitlines():
        if len(ln) < 4:
            continue
        p = ln[3:].strip()
        if (
            p.startswith("backend/routes/")
            or p.startswith("backend/services/")
            or p in {"backend/server.js", "backend/database/init.js"}
        ):
            lines.append(ln.strip())
    lines.sort()
    joined = "\n".join(lines)
    return hashlib.sha256(joined.encode("utf-8")).hexdigest()


def _run_compat() -> dict:
    r = _run(["python", "scripts/atlas_validate_cross_repo_compat.py"], cwd=BASE)
    out = (r.stdout or "").strip()
    payload = {
        "ts": _now_iso(),
        "ok": r.returncode == 0,
        "rc": r.returncode,
        "stdout": out[:1500],
        "stderr": (r.stderr or "")[:500],
    }
    _log(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Monitorea cambios de logica en panaderia y dispara validacion de compatibilidad."
    )
    parser.add_argument("--interval", type=int, default=20, help="Intervalo en segundos.")
    parser.add_argument("--once", action="store_true", help="Ejecuta un ciclo y termina.")
    args = parser.parse_args()

    last = _git_logic_fingerprint()
    _log({"ts": _now_iso(), "event": "watch_start", "fingerprint": last})

    # Primera validacion al iniciar.
    _run_compat()
    if args.once:
        return 0

    while True:
        time.sleep(max(5, int(args.interval)))
        cur = _git_logic_fingerprint()
        if cur != last:
            _log({"ts": _now_iso(), "event": "pan_logic_changed", "from": last, "to": cur})
            _run_compat()
            last = cur


if __name__ == "__main__":
    raise SystemExit(main())
