from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict


def _env_bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "y", "on")


def _run(cmd: list[str], cwd: Path) -> Dict[str, Any]:
    p = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True)
    return {
        "ok": p.returncode == 0,
        "code": p.returncode,
        "stdout": (p.stdout or "").strip(),
        "stderr": (p.stderr or "").strip(),
    }


def main() -> int:
    repo = Path.cwd()
    git_dir = repo / ".git"
    lock_path = git_dir / "index.lock"
    rebase_apply = git_dir / "rebase-apply"
    rebase_merge = git_dir / "rebase-merge"
    merge_head = git_dir / "MERGE_HEAD"

    out: Dict[str, Any] = {
        "ok": False,
        "repo": str(repo),
        "actions": [],
        "state": {},
    }

    # 1) Snapshot estado
    st = _run(["git", "status", "-sb"], cwd=repo)
    out["state"]["status_sb_ok"] = st["ok"]
    out["state"]["status_sb"] = (st.get("stdout") or "")[:800]

    # 2) Limpiar index.lock stale (solo si parece zombie)
    delete_lock = _env_bool("QUALITY_GIT_SAFE_SYNC_DELETE_LOCK", True)
    stale_sec = float(os.getenv("QUALITY_GIT_LOCK_STALE_SEC", "120") or "120")
    if lock_path.exists():
        try:
            age = time.time() - lock_path.stat().st_mtime
            size = lock_path.stat().st_size
            out["state"]["index_lock_age_sec"] = round(age, 1)
            out["state"]["index_lock_size"] = size
            if delete_lock and age >= stale_sec and size <= 16:
                lock_path.unlink(missing_ok=True)
                out["actions"].append({"action": "delete_index_lock", "ok": True, "age_sec": age, "size": size})
            else:
                out["actions"].append(
                    {
                        "action": "index_lock_present",
                        "ok": False,
                        "reason": "lock_not_stale_or_delete_disabled",
                        "age_sec": age,
                        "size": size,
                    }
                )
                print(json.dumps(out, ensure_ascii=False))
                return 2
        except Exception as e:
            out["actions"].append({"action": "delete_index_lock", "ok": False, "error": str(e)})
            print(json.dumps(out, ensure_ascii=False))
            return 2

    # 3) Detectar rebase/merge en progreso
    in_rebase = rebase_apply.exists() or rebase_merge.exists()
    in_merge = merge_head.exists()
    out["state"]["rebase_in_progress"] = in_rebase
    out["state"]["merge_in_progress"] = in_merge

    abort_rebase = _env_bool("QUALITY_GIT_SAFE_SYNC_ABORT_REBASE", False)
    abort_merge = _env_bool("QUALITY_GIT_SAFE_SYNC_ABORT_MERGE", False)

    if in_rebase:
        if abort_rebase:
            rr = _run(["git", "rebase", "--abort"], cwd=repo)
            out["actions"].append({"action": "rebase_abort", **rr})
            if not rr["ok"]:
                print(json.dumps(out, ensure_ascii=False))
                return 3
        else:
            out["actions"].append({"action": "rebase_in_progress", "ok": False, "reason": "abort_disabled"})
            print(json.dumps(out, ensure_ascii=False))
            return 3

    if in_merge:
        if abort_merge:
            mr = _run(["git", "merge", "--abort"], cwd=repo)
            out["actions"].append({"action": "merge_abort", **mr})
            if not mr["ok"]:
                print(json.dumps(out, ensure_ascii=False))
                return 3
        else:
            out["actions"].append({"action": "merge_in_progress", "ok": False, "reason": "abort_disabled"})
            print(json.dumps(out, ensure_ascii=False))
            return 3

    # 4) Si detached HEAD, no cambiar rama automáticamente (riesgo); reportar y salir.
    st2 = _run(["git", "status", "-sb"], cwd=repo)
    low = ((st2.get("stdout") or "") + "\n" + (st2.get("stderr") or "")).lower()
    if "head (no branch)" in low or "detached" in low:
        out["actions"].append({"action": "detached_head", "ok": False, "reason": "manual_intervention_required"})
        out["state"]["status_sb_after"] = (st2.get("stdout") or "")[:800]
        print(json.dumps(out, ensure_ascii=False))
        return 4

    # 5) Safe sync: fetch + pull ff-only
    do_pull = _env_bool("QUALITY_GIT_SAFE_SYNC_PULL", True)
    fr = _run(["git", "fetch", "origin"], cwd=repo)
    out["actions"].append({"action": "fetch_origin", **fr})
    if not fr["ok"]:
        print(json.dumps(out, ensure_ascii=False))
        return 5

    if do_pull:
        pr = _run(["git", "pull", "--ff-only"], cwd=repo)
        out["actions"].append({"action": "pull_ff_only", **pr})
        if not pr["ok"]:
            print(json.dumps(out, ensure_ascii=False))
            return 6

    out["ok"] = True
    print(json.dumps(out, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)

