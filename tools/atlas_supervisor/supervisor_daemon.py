from __future__ import annotations

import os
import sys
import time
import json
import hashlib
import subprocess
from datetime import datetime
from pathlib import Path

# Optional dependency: watchdog (recommended)
try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
    WATCHDOG_OK = True
except Exception:
    WATCHDOG_OK = False

ROOT = Path(os.getenv("ATLAS_WORKSPACE", r"C:\ATLAS_PUSH")).resolve()
REPORTS = ROOT / "reports"
LOGS = ROOT / "logs"
STATE_FILE = ROOT / ".atlas_supervisor_state.json"

WATCH_GLOBS = (".py", ".ps1", ".toml", ".json", ".yml", ".yaml", ".md", ".txt")
IGNORE_DIRS = {
    ".git",
    ".venv",
    "venv",
    "__pycache__",
    ".codex",
    "node_modules",
    ".temp_venv",
    "reports",
    "logs",
}
SMOKE_ON_EVENTS = {
    x.strip().lower()
    for x in os.getenv("ATLAS_SUPERVISOR_SMOKE_EVENTS", "startup").split(",")
    if x.strip()
}
SMOKE_MIN_INTERVAL_SEC = int(
    os.getenv("ATLAS_SUPERVISOR_SMOKE_MIN_INTERVAL_SEC", "600") or "600"
)
_last_smoke_ts: float = 0.0

REPORTS.mkdir(parents=True, exist_ok=True)
LOGS.mkdir(parents=True, exist_ok=True)

def now_id() -> str:
    return datetime.now().strftime("%Y%m%d-%H%M%S")

def sh(cmd: list[str], cwd: Path | None = None, timeout: int = 60) -> tuple[int, str, str]:
    p = subprocess.Popen(cmd, cwd=str(cwd or ROOT), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=False)
    try:
        out, err = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.kill()
        out, err = p.communicate()
        return 124, out, err + "\n[TIMEOUT]"
    return p.returncode, out, err

def file_hash(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def load_state() -> dict:
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text(encoding="utf-8", errors="ignore"))
        except Exception:
            return {}
    return {}

def save_state(state: dict) -> None:
    STATE_FILE.write_text(json.dumps(state, indent=2, ensure_ascii=False), encoding="utf-8")

def _is_under(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except Exception:
        return False

def is_relevant(path: Path) -> bool:
    if not path.exists():
        return False
    if path == STATE_FILE:
        return False
    if _is_under(path, REPORTS) or _is_under(path, LOGS):
        return False
    if any(part in IGNORE_DIRS for part in path.parts):
        return False
    if path.is_dir():
        return False
    return path.suffix.lower() in WATCH_GLOBS

def should_run_smoke(event: str) -> tuple[bool, str]:
    global _last_smoke_ts
    ev = (event or "").strip().lower()
    if ev not in SMOKE_ON_EVENTS:
        return False, f"Evento '{event}' no incluido en SMOKE_ON_EVENTS."
    now_ts = time.time()
    if _last_smoke_ts and (now_ts - _last_smoke_ts) < SMOKE_MIN_INTERVAL_SEC:
        wait = int(SMOKE_MIN_INTERVAL_SEC - (now_ts - _last_smoke_ts))
        return False, f"Cooldown activo ({wait}s restantes)."
    _last_smoke_ts = now_ts
    return True, "OK"

def scan_repo_quick() -> dict:
    """Quick scan: git status, key files existence, basic risks."""
    findings: list[str] = []
    info: dict = {
        "workspace": str(ROOT),
        "time": datetime.now().isoformat(timespec="seconds"),
        "git": {},
        "risks": [],
        "notes": [],
    }

    # Git status / branch
    rc, out, err = sh(["git", "rev-parse", "--is-inside-work-tree"], timeout=10)
    inside_git = (rc == 0 and out.strip().lower() == "true")
    info["git"]["inside"] = inside_git

    if inside_git:
        rc, out, err = sh(["git", "status", "--porcelain=v1"], timeout=20)
        info["git"]["dirty_files"] = out.strip().splitlines() if out.strip() else []
        rc, out, err = sh(["git", "rev-parse", "--abbrev-ref", "HEAD"], timeout=10)
        info["git"]["branch"] = out.strip() if rc == 0 else "unknown"

    # Key paths check (customize as needed)
    expected = [
        ROOT / "atlas_adapter",
        ROOT / "modules",
        ROOT / "bridge",
    ]
    missing = [str(p) for p in expected if not p.exists()]
    if missing:
        findings.append(f"Faltan rutas esperadas: {', '.join(missing)}")
        info["risks"].append("missing_key_paths")

    # Config checks
    codex_cfg = ROOT / ".codex" / "config.toml"
    if not codex_cfg.exists():
        findings.append("No existe .codex/config.toml en el workspace.")
        info["risks"].append("missing_codex_config")

    # Python env hint
    if not (ROOT / ".venv").exists() and not (ROOT / ".temp_venv").exists():
        info["notes"].append("No se detectó .venv/.temp_venv en el repo (puede estar fuera).")

    info["findings"] = findings
    return info

def try_run_smoke() -> tuple[bool, str]:
    """If your PowerShell smoke script exists, run it."""
    smoke_ps1 = ROOT / "04_smoke_tests.ps1"
    if not smoke_ps1.exists():
        return False, "No existe 04_smoke_tests.ps1 (skip)."
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
        timeout=180,
    )
    ok = (rc == 0)
    summary = (out + "\n" + err).strip()
    return ok, summary[-4000:]  # cap

def write_report(event: str, touched: list[str]) -> Path:
    rid = now_id()
    report_path = REPORTS / f"supervisor_report_{rid}.md"

    base = scan_repo_quick()
    run_smoke, smoke_reason = should_run_smoke(event)
    if run_smoke:
        smoke_ok, smoke_summary = try_run_smoke()
    else:
        smoke_ok = False
        smoke_summary = f"[SKIP] {smoke_reason}"

    lines: list[str] = []
    lines.append(f"# ATLAS Supervisor Report — {rid}")
    lines.append("")
    lines.append(f"**Evento:** {event}")
    lines.append(f"**Workspace:** `{base['workspace']}`")
    lines.append(f"**Hora:** `{base['time']}`")
    lines.append("")
    if touched:
        lines.append("## Archivos tocados (último evento)")
        for t in touched[:80]:
            lines.append(f"- `{t}`")
        lines.append("")

    lines.append("## Git")
    if base["git"].get("inside"):
        lines.append(f"- Branch: `{base['git'].get('branch','unknown')}`")
        dirty = base["git"].get("dirty_files") or []
        lines.append(f"- Cambios pendientes: **{len(dirty)}**")
        if dirty:
            lines.append("")
            lines.append("```")
            lines.extend(dirty[:200])
            lines.append("```")
    else:
        lines.append("- No es un repo git o git no disponible.")
    lines.append("")

    lines.append("## Hallazgos rápidos")
    f = base.get("findings") or []
    if not f:
        lines.append("- Sin hallazgos críticos en chequeo rápido.")
    else:
        for x in f:
            lines.append(f"- ⚠️ {x}")
    lines.append("")

    lines.append("## Smoke tests")
    lines.append(f"- Ejecutado: `{(ROOT / '04_smoke_tests.ps1').exists()}`")
    lines.append(f"- Trigger: `{event}`")
    lines.append(f"- Resultado: **{'OK' if smoke_ok else ('SKIP' if smoke_summary.startswith('[SKIP]') else 'NO/FAIL')}**")
    lines.append("")
    lines.append("```")
    lines.append(smoke_summary)
    lines.append("```")
    lines.append("")

    # Advice section
    lines.append("## Recomendaciones inmediatas (Supervisor)")
    if base.get("risks"):
        lines.append("- Revisar riesgos detectados y corregir antes de refactors grandes.")
    lines.append("- Si hay errores repetidos, priorizar: imports/rutas -> scripts PS1 -> dependencias -> runtime/puertos.")
    lines.append("- Mantener cambios pequeños y auditables (1 tema por commit).")
    lines.append("")

    report_path.write_text("\n".join(lines), encoding="utf-8")
    # --- AUTO-FIX EXECUTOR (safe) ---
    if touched:
        try:
            # run auto_fix on touched files (best-effort)
            cmd = ["python", "tools\\atlas_supervisor\\auto_fix.py"] + touched[:30]
            rc, out, err = sh(cmd, timeout=600)
            # append execution tail to the report (non-fatal)
            tail = (out + "\n" + err).strip()[-2000:]
            with report_path.open("a", encoding="utf-8") as f:
                f.write("\n## AutoFix Executor\n")
                f.write(f"- Ejecutado: `{rc == 0}` (rc={rc})\n\n")
                f.write("```text\n")
                f.write(tail + "\n")
                f.write("```\n")
        except Exception as ex:
            with report_path.open("a", encoding="utf-8") as f:
                f.write("\n## AutoFix Executor\n")
                f.write(f"- Error ejecutando auto_fix.py: {ex}\n")
    return report_path

class Handler(FileSystemEventHandler):
    def __init__(self) -> None:
        super().__init__()
        self.pending: set[str] = set()
        self.last_emit = 0.0

    def on_any_event(self, event):
        try:
            p = Path(getattr(event, "src_path", "")).resolve()
        except Exception:
            return
        if not is_relevant(p):
            return
        self.pending.add(str(p))
        # debounce
        if time.time() - self.last_emit > 3.0:
            self.flush("fs_event")

    def flush(self, reason: str):
        if not self.pending:
            return
        touched = sorted(self.pending)
        self.pending.clear()
        self.last_emit = time.time()
        rp = write_report(reason, touched)
        print(f"[SUPERVISOR] Report generado: {rp}")

def loop_poll(interval: int = 15):
    """Fallback if watchdog not installed: poll file hashes periodically."""
    state = load_state()
    prev = state.get("hashes", {})
    while True:
        touched = []
        hashes = {}
        for p in ROOT.rglob("*"):
            if not is_relevant(p):
                continue
            try:
                h = file_hash(p)
            except Exception:
                continue
            hashes[str(p)] = h
            if prev.get(str(p)) != h:
                touched.append(str(p))
        if touched:
            rp = write_report("poll_change", touched[:120])
            print(f"[SUPERVISOR] Report generado (poll): {rp}")
        prev = hashes
        save_state({"hashes": hashes, "time": datetime.now().isoformat(timespec="seconds")})
        time.sleep(interval)

def main():
    print(f"[SUPERVISOR] Workspace: {ROOT}")
    print(f"[SUPERVISOR] Watchdog: {'OK' if WATCHDOG_OK else 'NO (poll mode)'}")
    # initial report
    rp = write_report("startup", [])
    print(f"[SUPERVISOR] Report inicial: {rp}")

    if WATCHDOG_OK:
        observer = Observer()
        handler = Handler()
        observer.schedule(handler, str(ROOT), recursive=True)
        observer.start()
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()
    else:
        loop_poll(interval=15)

if __name__ == "__main__":
    sys.exit(main() or 0)
