from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

ROOT = Path(os.getenv("ATLAS_WORKSPACE", r"C:\ATLAS_PUSH")).resolve()
LOGS = ROOT / "logs"
REPORTS = ROOT / "reports"
BACKUPS = ROOT / ".atlas_backups"
RULES_MD = ROOT / "ATLAS_SUPERVISOR.md"

LOGS.mkdir(parents=True, exist_ok=True)
REPORTS.mkdir(parents=True, exist_ok=True)
BACKUPS.mkdir(parents=True, exist_ok=True)

SAFE_EXTS = {".py", ".ps1", ".toml", ".md", ".json", ".yml", ".yaml", ".txt"}
IGNORE_DIRS = {".git", ".venv", "venv", "__pycache__", "node_modules", ".codex", ".temp_venv", ".atlas_backups"}

@dataclass
class FixResult:
    ok: bool
    changed_files: list[str]
    notes: list[str]
    errors: list[str]

def now_id() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")

def sh(cmd: list[str], cwd: Path | None = None, timeout: int = 120) -> tuple[int, str, str]:
    p = subprocess.Popen(cmd, cwd=str(cwd or ROOT), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=False)
    try:
        out, err = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.kill()
        out, err = p.communicate()
        return 124, out, err + "\n[TIMEOUT]"
    return p.returncode, out, err

def is_ignored(p: Path) -> bool:
    return any(part in IGNORE_DIRS for part in p.parts)

def is_safe_file(p: Path) -> bool:
    return p.is_file() and (p.suffix.lower() in SAFE_EXTS) and not is_ignored(p)

def backup_file(p: Path) -> Path:
    rel = p.relative_to(ROOT)
    stamp = now_id()
    dest = BACKUPS / stamp / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dest)
    return dest

def read_text(p: Path) -> str:
    return p.read_text(encoding="utf-8", errors="ignore")

def write_text(p: Path, s: str) -> None:
    p.write_text(s, encoding="utf-8")

def normalize_line_endings(s: str) -> str:
    # keep as \n internally; Windows tools will handle
    s = s.replace("\r\n", "\n").replace("\r", "\n")
    return s

def ensure_trailing_newline(s: str) -> str:
    return s if s.endswith("\n") else (s + "\n")

def strip_trailing_spaces(s: str) -> str:
    return "\n".join([line.rstrip() for line in s.split("\n")])

def safe_text_normalize(p: Path) -> tuple[bool, str]:
    """Safe normalization: line endings, trailing spaces, final newline."""
    original = read_text(p)
    s = normalize_line_endings(original)
    s = strip_trailing_spaces(s)
    s = ensure_trailing_newline(s)
    if s != original:
        backup_file(p)
        write_text(p, s)
        return True, "normalized whitespace/newline"
    return False, "no change"

def ensure_utf8_bom_toml(p: Path) -> tuple[bool, str]:
    """
    VS Code shows "UTF-8 with BOM" sometimes. It's not required, but if
    your environment expects BOM (seen in your screenshots), we preserve it.
    We'll NOT add BOM; we only avoid breaking it by writing utf-8 text.
    """
    # python writes UTF-8 without BOM; that's usually OK.
    # We'll keep as "no-op" to avoid surprises.
    return False, "no-op (preserve encoding)"

def toml_sanity_codex_config(p: Path) -> tuple[bool, str]:
    """
    Minimal sanity for .codex/config.toml:
    - ensure required keys exist
    - avoid invalid empty assignments like `model_reasoning =`
    Does NOT remove your advanced settings.
    """
    txt = read_text(p)
    changed = False

    # Fix blank assignments: key =  (no value)
    def _fix_blank(key: str, default: str) -> None:
        nonlocal txt, changed
        pattern = re.compile(rf"(?m)^\s*{re.escape(key)}\s*=\s*$")
        if pattern.search(txt):
            txt = pattern.sub(f'{key} = "{default}"', txt)
            changed = True

    _fix_blank("model", "gpt-5.3-codex")
    # Codex variants show "model_reasoning_effort" sometimes; keep both possible:
    _fix_blank("model_reasoning", "high")
    _fix_blank("model_reasoning_effort", "high")
    _fix_blank("approval_policy", "on-request")
    _fix_blank("sandbox_mode", "workspace-write")

    # Ensure at least one reasoning key exists
    if "model_reasoning" not in txt and "model_reasoning_effort" not in txt:
        # add near model line
        m = re.search(r'(?m)^\s*model\s*=\s*".+?"\s*$', txt)
        insert = '\nmodel_reasoning_effort = "high"\n'
        if m:
            idx = m.end()
            txt = txt[:idx] + insert + txt[idx:]
        else:
            txt = 'model = "gpt-5.3-codex"\nmodel_reasoning_effort = "high"\n' + txt
        changed = True

    if changed:
        backup_file(p)
        write_text(p, txt)
        return True, "codex config sanity applied"
    return False, "no change"

def ensure_powershell_host_var(p: Path) -> tuple[bool, str]:
    """
    Safe ATLAS-specific fix:
    Replace PowerShell param $Host collisions by $BindHost if detected.
    (You already hit this before.)
    """
    txt = read_text(p)
    if "$Host" not in txt:
        return False, "no $Host usage"
    # Only change in param blocks / variable declarations to avoid breaking strings
    # conservative: replace "$Host" tokens when it's a variable reference.
    new = re.sub(r"(?<![\w])\$Host(?![\w])", "$BindHost", txt)
    if new != txt:
        backup_file(p)
        write_text(p, new)
        return True, "replaced $Host -> $BindHost (safe)"
    return False, "no change"

def ensure_git_autocrlf_note() -> tuple[bool, str]:
    """
    Non-invasive: just report if autocrlf may cause diffs.
    Does not change git config automatically.
    """
    rc, out, err = sh(["git", "config", "--get", "core.autocrlf"], timeout=10)
    v = out.strip() if rc == 0 else ""
    if v.lower() in {"true", "input", "false"}:
        return False, f"git core.autocrlf={v}"
    return False, "git core.autocrlf not set (ok)"

def run_python_formatters() -> tuple[bool, str]:
    """
    Optional: run ruff/black if present.
    Safe: only runs if tool is installed; otherwise no-op.
    """
    notes = []
    changed = False

    # ruff
    rc, out, err = sh(["python", "-m", "ruff", "--version"], timeout=10)
    if rc == 0:
        rc2, out2, err2 = sh(["python", "-m", "ruff", "check", ".", "--fix"], timeout=180)
        notes.append("ruff --fix executed" if rc2 == 0 else f"ruff --fix issues: rc={rc2}")
        changed = changed or (rc2 == 0)
    else:
        notes.append("ruff not installed (skip)")

    # black
    rc, out, err = sh(["python", "-m", "black", "--version"], timeout=10)
    if rc == 0:
        rc2, out2, err2 = sh(["python", "-m", "black", "."], timeout=240)
        notes.append("black executed" if rc2 == 0 else f"black issues: rc={rc2}")
        changed = changed or (rc2 == 0)
    else:
        notes.append("black not installed (skip)")

    return changed, "; ".join(notes)

def smoke_tests_if_exists() -> tuple[bool, str]:
    smoke_ps1 = ROOT / "04_smoke_tests.ps1"
    if not smoke_ps1.exists():
        return False, "no smoke script (skip)"
    rc, out, err = sh(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(smoke_ps1),
            "-RepoPath",
            str(ROOT),
        ],
        timeout=240,
    )
    ok = (rc == 0)
    tail = (out + "\n" + err).strip()[-4000:]
    return ok, ("SMOKE OK" if ok else "SMOKE FAIL") + "\n" + tail

def write_autofix_report(result: FixResult, header: str, extra: list[str]) -> Path:
    rid = now_id()
    p = REPORTS / f"autofix_{rid}.md"
    lines = []
    lines.append(f"# ATLAS AutoFix Report — {rid}")
    lines.append("")
    lines.append(f"**Workspace:** `{ROOT}`")
    lines.append(f"**Header:** {header}")
    lines.append("")
    if RULES_MD.exists():
        lines.append(f"**Reglas:** `{RULES_MD}` (leídas por política, no modificadas)")
        lines.append("")

    lines.append("## Cambios aplicados")
    if result.changed_files:
        for f in result.changed_files:
            lines.append(f"- ✅ `{f}`")
    else:
        lines.append("- (sin cambios)")
    lines.append("")

    lines.append("## Notas")
    for n in (result.notes + extra):
        lines.append(f"- {n}")
    lines.append("")

    if result.errors:
        lines.append("## Errores")
        for e in result.errors:
            lines.append(f"- ❌ {e}")
        lines.append("")

    p.write_text("\n".join(lines), encoding="utf-8")
    return p

def apply_safe_autofixes(targets: Iterable[Path]) -> FixResult:
    changed_files: list[str] = []
    notes: list[str] = []
    errors: list[str] = []

    # 1) Always do gentle normalization on touched files
    for t in targets:
        try:
            if not is_safe_file(t):
                continue
            changed, msg = safe_text_normalize(t)
            if changed:
                changed_files.append(str(t.relative_to(ROOT)))
                notes.append(f"{t.name}: {msg}")
        except Exception as ex:
            errors.append(f"{t}: normalize failed: {ex}")

    # 2) ATLAS-specific safe PS1 fix: $Host collision
    for t in targets:
        try:
            if t.suffix.lower() == ".ps1" and t.exists() and not is_ignored(t):
                changed, msg = ensure_powershell_host_var(t)
                if changed:
                    changed_files.append(str(t.relative_to(ROOT)))
                    notes.append(f"{t.name}: {msg}")
        except Exception as ex:
            errors.append(f"{t}: ps1 fix failed: {ex}")

    # 3) Codex config sanity if touched or present
    codex_cfg = ROOT / ".codex" / "config.toml"
    try:
        if codex_cfg.exists():
            changed, msg = toml_sanity_codex_config(codex_cfg)
            if changed:
                changed_files.append(str(codex_cfg.relative_to(ROOT)))
                notes.append(msg)
    except Exception as ex:
        errors.append(f"codex config sanity failed: {ex}")

    # 4) Optional: run python formatters (if installed)
    try:
        changed, msg = run_python_formatters()
        notes.append(msg)
    except Exception as ex:
        errors.append(f"formatters failed: {ex}")

    # 5) Smoke tests if available
    smoke_ok = False
    try:
        smoke_ok, msg = smoke_tests_if_exists()
        notes.append(msg)
        if not smoke_ok and "FAIL" in msg:
            notes.append("SMOKE FAIL: revisar reportes y revertir cambios si aplica.")
    except Exception as ex:
        errors.append(f"smoke tests failed: {ex}")

    # 6) Auto-commit seguro (solo si smoke_ok)
    committed_ok = False
    try:
        committed_ok, cmsg = safe_autocommit(changed_files, smoke_ok)
        notes.append(cmsg)
    except Exception as ex:
        errors.append(f"auto-commit failed: {ex}")

    # 7) Auto-push seguro (solo si committed_ok)
    try:
        pushed_ok, pmsg = safe_autopush(committed_ok)
        notes.append(pmsg)
    except Exception as ex:
        errors.append(f"auto-push failed: {ex}")

    return FixResult(ok=(len(errors) == 0), changed_files=sorted(set(changed_files)), notes=notes, errors=errors)

def git_ok() -> bool:
    rc, out, _ = sh(["git", "rev-parse", "--is-inside-work-tree"], timeout=10)
    return rc == 0 and out.strip().lower() == "true"

def git_busy() -> bool:
    # evita commits durante merge/rebase/cherry-pick
    git_dir = ROOT / ".git"
    flags = ["MERGE_HEAD", "rebase-apply", "rebase-merge", "CHERRY_PICK_HEAD"]
    for f in flags:
        if (git_dir / f).exists():
            return True
    return False

def git_branch() -> str:
    rc, out, _ = sh(["git", "rev-parse", "--abbrev-ref", "HEAD"], timeout=10)
    return out.strip() if rc == 0 else "unknown"

def working_tree_dirty() -> bool:
    rc, out, _ = sh(["git", "status", "--porcelain=v1"], timeout=15)
    return rc == 0 and bool(out.strip())

def safe_autocommit(changed_rel_files: list[str], smoke_ok: bool) -> tuple[bool, str]:
    """
    Auto-commit guardrails:
    - Solo si smoke_ok
    - Solo si git OK y no busy
    - Solo si cambios pequeños (max 25 archivos) y extensiones seguras
    """
    if not AUTO_COMMIT_ENABLED:
        return False, "Auto-commit: disabled by ATLAS_AUTOCOMMIT=0"
    if not smoke_ok:
        return False, "Auto-commit: skip (SMOKE no OK)."
    if not git_ok():
        return False, "Auto-commit: skip (no es repo git)."
    if git_busy():
        return False, "Auto-commit: skip (git ocupado: merge/rebase/cherry-pick)."
    if not working_tree_dirty():
        return False, "Auto-commit: skip (no hay cambios para commitear)."

    # Guardrail: limitar tamaño
    unique_files = sorted(set(changed_rel_files))
    if len(unique_files) == 0:
        return False, "Auto-commit: skip (lista de cambios vacía)."
    if len(unique_files) > 25:
        return False, f"Auto-commit: skip (demasiados archivos: {len(unique_files)} > 25)."

    # Guardrail: solo archivos seguros
    bad = []
    for rel in unique_files:
        p = (ROOT / rel).resolve()
        if not p.exists():
            continue
        if p.suffix.lower() not in SAFE_EXTS:
            bad.append(rel)
    if bad:
        return False, f"Auto-commit: skip (archivos no seguros detectados: {bad[:10]})."

    # Stage solo lo que tocamos (si existe); si no, stage todo seguro del status
    for rel in unique_files:
        p = ROOT / rel
        if p.exists():
            sh(["git", "add", "--", rel], timeout=30)

    # Mensaje estándar tipo “Cursor”
    br = git_branch()
    short_list = ", ".join(unique_files[:6]) + ("..." if len(unique_files) > 6 else "")
    msg = f"chore(atlas): supervisor autofix (smoke ok) [{br}]"

    # Commit
    rc, out, err = sh(["git", "commit", "-m", msg, "-m", f"files: {short_list}"], timeout=60)
    if rc == 0:
        return True, f"Auto-commit: OK ({msg})"
    # Si no hay cambios staged, git devuelve nonzero; reportar
    tail = (out + "\n" + err).strip()[-1500:]
    return False, "Auto-commit: FAIL\n" + tail

PROTECTED_BRANCHES = {"main", "master", "release", "prod", "production"}
AUTO_PUSH_ENABLED = os.getenv("ATLAS_AUTOPUSH", "1").strip() not in {"0", "false", "no", "off"}
AUTO_COMMIT_ENABLED = os.getenv("ATLAS_AUTOCOMMIT", "1").strip() not in {"0", "false", "no", "off"}
FORCE_DEV_BRANCH = os.getenv("ATLAS_FORCE_DEV", "1").strip() not in {"0", "false", "no", "off"}

def git_has_origin() -> bool:
    rc, out, _ = sh(["git", "remote"], timeout=10)
    remotes = set([x.strip() for x in out.splitlines() if x.strip()])
    return "origin" in remotes

def git_current_branch() -> str:
    return git_branch()

def safe_autopush(committed_ok: bool) -> tuple[bool, str]:
    """
    Auto-push guardrails:
    - Solo si se realizó commit OK en este ciclo
    - Solo si existe remote origin
    - Nunca push en ramas protegidas
    - Push a origin <branch>
    """
    if not AUTO_PUSH_ENABLED:
        return False, "Auto-push: disabled by ATLAS_AUTOPUSH=0"
    if not committed_ok:
        return False, "Auto-push: skip (no hubo commit en este ciclo)."
    if not git_ok():
        return False, "Auto-push: skip (no es repo git)."
    if git_busy():
        return False, "Auto-push: skip (git ocupado: merge/rebase/cherry-pick)."
    if not git_has_origin():
        return False, "Auto-push: skip (remote 'origin' no existe)."

    br = git_current_branch()
    if FORCE_DEV_BRANCH and br != "dev":
        return False, f"Auto-push: BLOCKED (no estás en 'dev', estás en '{br}'). Cambia a dev o pon ATLAS_FORCE_DEV=0."
    if br in PROTECTED_BRANCHES:
        return False, f"Auto-push: BLOCKED (rama protegida: {br})."

    # Push
    rc, out, err = sh(["git", "push", "-u", "origin", br], timeout=120)
    if rc == 0:
        return True, f"Auto-push: OK (origin/{br})"
    tail = (out + "\n" + err).strip()[-1500:]
    return False, "Auto-push: FAIL\n" + tail

def main():
    # Targets: optional args as file paths; if none, do a quick safe sweep on key config files
    args = [Path(a).resolve() for a in os.sys.argv[1:]]
    targets = [a for a in args if a.exists()] if args else []

    # If no targets, use a small default set (safe)
    if not targets:
        defaults = [
            ROOT / ".codex" / "config.toml",
            ROOT / "03_run_atlas_api.ps1",
            ROOT / "04_smoke_tests.ps1",
        ]
        targets = [p for p in defaults if p.exists()]

    header = "auto_fix invoked"
    result = apply_safe_autofixes(targets)

    extra = []
    try:
        _, msg = ensure_git_autocrlf_note()
        extra.append(msg)
    except Exception:
        pass

    rp = write_autofix_report(result, header, extra)
    print(f"[AUTOFIX] Report: {rp}")
    if result.errors:
        print("[AUTOFIX] Errors present. See report.")
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
