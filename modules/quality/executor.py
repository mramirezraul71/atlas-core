from __future__ import annotations

import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .models import POT, POTResult
from .policy import GitPolicy, filter_paths_for_commit, get_git_policy, repo_root


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _run(cmd: List[str], cwd: Path, timeout: int = 45) -> Tuple[bool, str]:
    p = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True, timeout=timeout)
    out = ((p.stdout or "") + "\n" + (p.stderr or "")).strip()
    return p.returncode == 0, out


def _git_status_porcelain(cwd: Path) -> List[str]:
    ok, out = _run(["git", "status", "--porcelain"], cwd=cwd, timeout=20)
    if not ok:
        return []
    lines = [ln for ln in out.splitlines() if ln.strip()]
    return lines


def _parse_porcelain_paths(lines: List[str]) -> List[str]:
    paths: List[str] = []
    for ln in lines:
        # format: "XY path" o "XY path -> new"
        parts = ln.strip().split(maxsplit=1)
        if len(parts) < 2:
            continue
        p = parts[1].strip()
        if " -> " in p:
            p = p.split(" -> ", 1)[-1].strip()
        paths.append(p)
    return paths


def _git_ahead_behind(cwd: Path) -> Tuple[int, int, str]:
    ok, out = _run(["git", "status", "-sb"], cwd=cwd, timeout=20)
    if not ok:
        return 0, 0, ""
    first = (out.splitlines() or [""])[0]
    # Ej: "## branch...origin/branch [ahead 1, behind 2]"
    ahead = 0
    behind = 0
    if "ahead" in first:
        try:
            ahead = int(first.split("ahead", 1)[1].split(",", 1)[0].strip().split()[0].strip("]"))
        except Exception:
            ahead = 0
    if "behind" in first:
        try:
            behind = int(first.split("behind", 1)[1].split("]", 1)[0].strip().split()[0])
        except Exception:
            behind = 0
    return ahead, behind, first.strip()


def run_pot(pot: POT, context: Optional[Dict] = None) -> POTResult:
    ctx = dict(context or {})
    step_outputs: List[Dict] = []
    buf: List[str] = []
    ok_all = True

    for step in pot.steps:
        try:
            out = step.run(ctx)
            step_outputs.append({"id": step.id, "name": step.name, "ok": True, "output": out})
            buf.append(f"[OK] {step.name}\n{out}".rstrip())
        except Exception as e:
            ok_all = False
            step_outputs.append({"id": step.id, "name": step.name, "ok": False, "error": str(e)})
            buf.append(f"[FAIL] {step.name}\n{e}".rstrip())
            if step.fatal:
                break

    report_path = _write_report(pot, ok_all, step_outputs)
    return POTResult(pot_id=pot.id, ok=ok_all, output="\n\n".join(buf).strip(), step_outputs=step_outputs, report_path=report_path)


def _write_report(pot: POT, ok: bool, step_outputs: List[Dict]) -> str:
    root = repo_root()
    reports_dir = root / "logs" / "pots"
    reports_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    path = reports_dir / f"pot_{pot.id}_{ts}.json"
    payload = {
        "ts": _now(),
        "pot_id": pot.id,
        "pot_name": pot.name,
        "ok": ok,
        "steps": step_outputs,
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return str(path)


# -----------------------------------------------------------------------------
# POT helper: Git Safe Sync
# -----------------------------------------------------------------------------


def git_safe_sync(mode: str = "check") -> str:
    """
    Ejecuta ciclo Git seguro.
    mode:
      - check: solo evidencia (fetch+status), sin modificar repo
      - sync: permite pull/push/commit SOLO si policy lo permite explícitamente
    """
    root = repo_root()
    policy = get_git_policy()

    # Seguridad: no permitir "sync" en detached HEAD o durante rebase.
    # (check-only siempre permitido para evidencia)
    if mode == "sync":
        try:
            okb, outb = _run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=root, timeout=10)
            branch = (outb or "").strip().splitlines()[-1] if outb else ""
            if not okb or branch == "HEAD":
                raise RuntimeError("SYNC BLOQUEADO: repo en detached HEAD (no branch).")
        except RuntimeError:
            raise
        except Exception:
            raise RuntimeError("SYNC BLOQUEADO: no se pudo determinar branch actual.")
        try:
            if (root / ".git" / "rebase-merge").exists() or (root / ".git" / "rebase-apply").exists():
                raise RuntimeError("SYNC BLOQUEADO: rebase en progreso detectado.")
        except RuntimeError:
            raise
        except Exception:
            # Si no podemos leer .git, preferir bloquear.
            raise RuntimeError("SYNC BLOQUEADO: no se pudo validar estado de rebase.")

    if policy.forbid_rebase:
        # Política dura (documental): no ejecutamos rebase, y además lo declaramos.
        pass

    ok, out = _run(["git", "rev-parse", "--is-inside-work-tree"], cwd=root, timeout=10)
    if not ok or "true" not in out.lower():
        raise RuntimeError("No es un repositorio Git válido en ATLAS_PUSH.")

    _run(["git", "fetch", "--quiet"], cwd=root, timeout=30)
    ahead, behind, header = _git_ahead_behind(root)

    porcelain = _git_status_porcelain(root)
    paths = _parse_porcelain_paths(porcelain)
    commit_candidates = filter_paths_for_commit(paths, policy)

    lines = []
    lines.append("GIT SAFE SYNC (SIN REBASE)")
    lines.append(f"- mode={mode}")
    lines.append(f"- status={header}")
    lines.append(f"- ahead={ahead} behind={behind}")
    lines.append(f"- changed={len(paths)} (commit_candidates={len(commit_candidates)})")
    if paths:
        lines.append("- changed_paths:")
        for p in paths[:30]:
            lines.append(f"  - {p}")
        if len(paths) > 30:
            lines.append(f"  ... +{len(paths) - 30} más")

    if mode != "sync":
        lines.append("- acción: CHECK ONLY (sin cambios)")
        return "\n".join(lines)

    # sync mode (requiere flags explícitos)
    if behind > 0:
        if not policy.allow_pull:
            lines.append("- pull: BLOQUEADO (QUALITY_GIT_ALLOW_PULL=false)")
        else:
            if policy.require_clean_tree_for_pull and paths:
                lines.append("- pull: BLOQUEADO (working tree no está limpio)")
            else:
                okp, outp = _run(["git", "pull", "--ff-only"], cwd=root, timeout=60)
                lines.append(f"- pull: {'OK' if okp else 'FAIL'}")
                if outp:
                    lines.append(outp[:500])

    if commit_candidates:
        if not policy.allow_commit:
            lines.append("- commit: BLOQUEADO (QUALITY_GIT_ALLOW_COMMIT=false)")
        else:
            # stage solo candidatos permitidos
            for p in commit_candidates:
                _run(["git", "add", "--", p], cwd=root, timeout=30)
            msg = f"chore(repo): safe autosync ({datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S')}Z)"
            okc, outc = _run(["git", "commit", "-m", msg], cwd=root, timeout=60)
            lines.append(f"- commit: {'OK' if okc else 'FAIL'}")
            if outc:
                lines.append(outc[:500])

    # push
    porcelain2 = _git_status_porcelain(root)
    paths2 = _parse_porcelain_paths(porcelain2)
    ahead2, behind2, header2 = _git_ahead_behind(root)
    if policy.require_clean_tree_for_push and paths2:
        lines.append("- push: BLOQUEADO (working tree no está limpio)")
        return "\n".join(lines)

    if ahead2 > 0:
        if not policy.allow_push:
            lines.append("- push: BLOQUEADO (QUALITY_GIT_ALLOW_PUSH=false)")
        else:
            okps, outps = _run(["git", "push"], cwd=root, timeout=90)
            lines.append(f"- push: {'OK' if okps else 'FAIL'}")
            if outps:
                lines.append(outps[:800])
    else:
        lines.append("- push: no necesario (ahead=0)")

    lines.append(f"- status_final={header2}")
    return "\n".join(lines)

