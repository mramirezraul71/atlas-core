from __future__ import annotations

import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict

INTERNAL_HOOK_TAG = "[ATLAS_HOOK_AUTO]"
GIT_TIMEOUT_SECONDS = 12


def _repo_root() -> Path:
    # scripts/ -> repo root
    return Path(__file__).resolve().parent.parent


def _log_file(repo: Path) -> Path:
    logs_dir = repo / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    return logs_dir / "git_post_commit.log"


def _get_logger(repo: Path) -> logging.Logger:
    logger = logging.getLogger("atlas.git_post_commit")
    if logger.handlers:
        return logger

    logger.setLevel(logging.INFO)
    handler = logging.FileHandler(_log_file(repo), encoding="utf-8")
    fmt = logging.Formatter("[%(asctime)s] [%(levelname)s] %(message)s")
    handler.setFormatter(fmt)
    logger.addHandler(handler)
    logger.propagate = False
    return logger


def _git(repo: Path, *args: str, timeout_s: int = 10) -> Dict[str, str]:
    try:
        r = subprocess.run(
            ["git", *args],
            cwd=str(repo),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            env={**os.environ, "LANG": "C"},
        )
        return {
            "ok": "1" if r.returncode == 0 else "0",
            "stdout": (r.stdout or "").strip(),
            "stderr": (r.stderr or "").strip(),
        }
    except subprocess.TimeoutExpired:
        return {
            "ok": "0",
            "stdout": "",
            "stderr": f"git timeout ({timeout_s}s) args={args}",
        }
    except Exception as e:
        return {
            "ok": "0",
            "stdout": "",
            "stderr": f"git exception {type(e).__name__}: {e}",
        }


def main() -> int:
    repo = _repo_root()
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    logger = _get_logger(repo)

    if os.getenv("ATLAS_GIT_HOOKS_ENABLED", "1") != "1":
        logger.info("Hook deshabilitado por ATLAS_GIT_HOOKS_ENABLED != 1")
        return 0

    if not (repo / ".git").exists():
        logger.info("No se ejecuta hook: no existe .git en %s", repo)
        return 0

    try:
        head = _git(repo, "rev-parse", "--short", "HEAD", timeout_s=GIT_TIMEOUT_SECONDS)
        subj = _git(repo, "log", "-1", "--pretty=%s", timeout_s=GIT_TIMEOUT_SECONDS)
        files = _git(
            repo,
            "show",
            "--name-only",
            "--pretty=",
            timeout_s=GIT_TIMEOUT_SECONDS,
        )

        commit = (head.get("stdout") or "").strip()[:12]
        subject = (subj.get("stdout") or "").strip()
        changed = len(
            [ln for ln in (files.get("stdout") or "").splitlines() if ln.strip()]
        )

        # Anti-loop: si el commit fue generado por automatismo del hook, abortar.
        if INTERNAL_HOOK_TAG in subject:
            logger.info("Hook abortado por anti-loop. subject='%s'", subject[:200])
            return 0

        # Mensaje humano (sin IDs/rutas)
        msg = "Nuevo cambio guardado en Git."
        if subject:
            msg += f" Resumen: {subject}."
        if changed:
            msg += f" Archivos actualizados: {changed}."

        try:
            from modules.humanoid.comms.ops_bus import emit

            emit(
                "repo",
                msg,
                level="info",
                data={"commit": commit, "changed_files": changed},
            )
        except Exception as e:
            logger.warning("emit falló: %s: %s", type(e).__name__, e)

        try:
            from modules.humanoid.ans.evolution_bitacora import \
                append_evolution_log

            append_evolution_log(
                f"[REPO] Commit creado. Resumen: {subject[:120]}",
                ok=True,
                source="repo",
            )
        except Exception as e:
            logger.warning("append_evolution_log falló: %s: %s", type(e).__name__, e)

        logger.info(
            "Hook ejecutado commit=%s changed=%s subject='%s'",
            commit or "unknown",
            changed,
            subject[:200],
        )
        return 0
    except Exception as e:
        # Nunca bloquear commit.
        logger.exception(
            "Fallo global en git_post_commit_hook: %s: %s", type(e).__name__, e
        )
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
