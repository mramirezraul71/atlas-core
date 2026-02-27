from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Dict


def _repo_root() -> Path:
    # scripts/ -> repo root
    return Path(__file__).resolve().parent.parent


def _git(repo: Path, *args: str, timeout_s: int = 10) -> Dict[str, str]:
    r = subprocess.run(
        ["git", *args],
        cwd=str(repo),
        capture_output=True,
        text=True,
        timeout=timeout_s,
        env={**os.environ, "LANG": "C"},
    )
    return {"ok": "1" if r.returncode == 0 else "0", "stdout": (r.stdout or "").strip(), "stderr": (r.stderr or "").strip()}


def main() -> int:
    repo = _repo_root()
    if not (repo / ".git").exists():
        return 0
    head = _git(repo, "rev-parse", "--short", "HEAD")
    subj = _git(repo, "log", "-1", "--pretty=%s")
    files = _git(repo, "show", "--name-only", "--pretty=")

    commit = (head.get("stdout") or "").strip()[:12]
    subject = (subj.get("stdout") or "").strip()
    changed = len([ln for ln in (files.get("stdout") or "").splitlines() if ln.strip()])

    # Mensaje humano (sin IDs/rutas)
    msg = "Nuevo cambio guardado en Git."
    if subject:
        msg += f" Resumen: {subject}."
    if changed:
        msg += f" Archivos actualizados: {changed}."

    try:
        from modules.humanoid.comms.ops_bus import emit

        emit("repo", msg, level="info", data={"commit": commit, "changed_files": changed})
    except Exception:
        pass

    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log

        append_evolution_log(f"[REPO] Commit creado. Resumen: {subject[:120]}", ok=True, source="repo")
    except Exception:
        pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

