from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Dict


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


_POLICY_PATH = (_repo_root() / "logs" / "supervisor_policy.txt").resolve()
_POLICY_PATH.parent.mkdir(parents=True, exist_ok=True)


def get_supervisor_policy() -> Dict[str, object]:
    """
    Returns current supervisor resident policy stored in logs/ (local, ignored by git).
    """
    try:
        if not _POLICY_PATH.exists():
            return {
                "ok": True,
                "policy": "",
                "updated_at": 0.0,
                "path": str(_POLICY_PATH),
            }
        st = _POLICY_PATH.stat()
        txt = _POLICY_PATH.read_text(encoding="utf-8", errors="ignore")
        return {
            "ok": True,
            "policy": txt,
            "updated_at": float(st.st_mtime),
            "path": str(_POLICY_PATH),
        }
    except Exception as e:
        return {
            "ok": False,
            "policy": "",
            "updated_at": 0.0,
            "path": str(_POLICY_PATH),
            "error": str(e)[:200],
        }


def set_supervisor_policy(policy_text: str) -> Dict[str, object]:
    """
    Persist supervisor resident policy to logs/ (local, ignored by git).
    """
    try:
        txt = (policy_text or "").strip()
        # Guardrails: avoid accidental huge dumps
        if len(txt) > 50_000:
            txt = txt[:50_000]
        _POLICY_PATH.write_text(txt + "\n", encoding="utf-8")
        now = time.time()
        try:
            os.utime(_POLICY_PATH, (now, now))
        except Exception:
            pass
        return {"ok": True, "policy": txt, "updated_at": now, "path": str(_POLICY_PATH)}
    except Exception as e:
        return {
            "ok": False,
            "policy": "",
            "updated_at": 0.0,
            "path": str(_POLICY_PATH),
            "error": str(e)[:200],
        }
