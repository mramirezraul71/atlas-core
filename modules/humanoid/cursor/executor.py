"""Cursor step executor: ejecuta acciones reales con bitácora.

Problema que resuelve:
- Antes, `mode=auto` marcaba muchos pasos como OK aunque fueran "skipped".
- No había transcript de terminal ni scripts reproducibles.

Este ejecutor genera:
- transcript en memoria (preview) + archivo log
- script .ps1 con comandos ejecutados (cuando aplica)
- JSON de run con resultados por paso
"""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from modules.humanoid.hands.safe_shell import SafeShellExecutor


def _now_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve()
    # modules/humanoid/cursor/executor.py -> repo_root
    return Path(__file__).resolve().parents[3]


def _cursor_snapshots_dir(repo_root: Optional[Path] = None) -> Path:
    r = (repo_root or _repo_root()).resolve()
    d = r / "snapshots" / "cursor"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(content, encoding="utf-8", errors="ignore")
    os.replace(str(tmp), str(path))


def _safe_filename(s: str, max_len: int = 48) -> str:
    s = (s or "").strip().lower()
    s = re.sub(r"[^a-z0-9_-]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    return (s or "run")[:max_len]


@dataclass
class RunArtifacts:
    run_id: str
    run_dir: Path
    log_path: Path
    script_path: Path
    json_path: Path


class CursorExecutor:
    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()
        self.shell = SafeShellExecutor()

    def new_run_artifacts(self, goal: str) -> RunArtifacts:
        ts = _now_stamp()
        rid = f"cursor_{ts}_{_safe_filename(goal)}"
        run_dir = _cursor_snapshots_dir(self.repo_root) / rid
        run_dir.mkdir(parents=True, exist_ok=True)
        artifacts = RunArtifacts(
            run_id=rid,
            run_dir=run_dir,
            log_path=run_dir / "terminal.log",
            script_path=run_dir / "run.ps1",
            json_path=run_dir / "run.json",
        )
        # Precrear log y script para que SIEMPRE existan como evidencia.
        try:
            if not artifacts.log_path.exists():
                artifacts.log_path.write_text("", encoding="utf-8")
        except Exception:
            pass
        try:
            if not artifacts.script_path.exists():
                header = (
                    "# Cursor-ATLAS script (auto generado)\n"
                    "# Ejecuta en PowerShell desde el root del repo.\n"
                    "Set-StrictMode -Version Latest\n"
                    "$ErrorActionPreference = 'Stop'\n\n"
                )
                artifacts.script_path.write_text(header, encoding="utf-8")
        except Exception:
            pass
        return artifacts

    def _append_log(self, artifacts: RunArtifacts, text: str) -> None:
        try:
            artifacts.log_path.parent.mkdir(parents=True, exist_ok=True)
            with artifacts.log_path.open("a", encoding="utf-8", errors="ignore") as f:
                f.write(text.rstrip() + "\n")
        except Exception:
            pass

    def _append_script(self, artifacts: RunArtifacts, cmd: str) -> None:
        """Script reproducible (PowerShell) con comandos ejecutados."""
        try:
            artifacts.script_path.parent.mkdir(parents=True, exist_ok=True)
            if not artifacts.script_path.exists():
                header = (
                    "# Cursor-ATLAS script (auto generado)\n"
                    "# Ejecuta en PowerShell desde el root del repo.\n"
                    "Set-StrictMode -Version Latest\n"
                    "$ErrorActionPreference = 'Stop'\n\n"
                )
                artifacts.script_path.write_text(header, encoding="utf-8")
            with artifacts.script_path.open("a", encoding="utf-8", errors="ignore") as f:
                f.write("\n# --- step command ---\n")
                f.write(cmd.rstrip() + "\n")
        except Exception:
            pass

    def _run_shell(self, artifacts: RunArtifacts, cmd: str, timeout_s: int = 60) -> Dict[str, Any]:
        self._append_log(artifacts, f"$ {cmd}")
        res = self.shell.run(cmd, cwd=str(self.repo_root), timeout_sec=int(timeout_s or 60))
        out = (res.get("stdout") or "").strip()
        err = (res.get("stderr") or "").strip()
        if out:
            self._append_log(artifacts, out)
        if err:
            self._append_log(artifacts, "STDERR:\n" + err)
        self._append_script(artifacts, cmd)
        return res

    def _run_status(self) -> Dict[str, Any]:
        try:
            from modules.command_router import handle
            return {"ok": True, "output": handle("/status")}
        except Exception as e:
            return {"ok": False, "output": "", "error": str(e)}

    def _run_health(self) -> Dict[str, Any]:
        try:
            from modules.humanoid.deploy.healthcheck import run_health_verbose
            return run_health_verbose(base_url=None)
        except Exception as e:
            return {"ok": False, "score": 0, "checks": {}, "error": str(e)}

    def _run_camera_health(self) -> Dict[str, Any]:
        try:
            from modules.humanoid.ans.checks.robot_camera_health import run
            return run()
        except Exception as e:
            return {"ok": False, "check_id": "robot_camera_health", "message": str(e), "severity": "med"}

    def execute_step(self, artifacts: RunArtifacts, step: Dict[str, Any]) -> Dict[str, Any]:
        """Ejecuta 1 step. Devuelve estructura rica para UI."""
        desc = (step.get("description") or "").strip()
        low = desc.lower()
        result: Dict[str, Any] = {
            "description": desc,
            "ok": False,
            "action": None,
            "details": None,
            "terminal_log_path": str(artifacts.log_path),
            "script_path": str(artifacts.script_path),
        }

        # 1) Diagnóstico (status/health)
        if "/status" in low or ("verificar estado" in low and "sistema" in low) or low.startswith("status"):
            out = self._run_status()
            result["action"] = "status"
            result["details"] = out
            result["ok"] = bool(out.get("ok"))
            self._append_log(artifacts, "STATUS:\n" + str(out.get("output") or out))
            return result

        if "/health" in low or "salud" in low or "health" in low:
            out = self._run_health()
            result["action"] = "health"
            result["details"] = out
            result["ok"] = bool(out.get("ok"))
            self._append_log(artifacts, "HEALTH:\n" + json.dumps(out, ensure_ascii=False)[:4000])
            return result

        # 2) Evidencia rápida (screenshot)
        if "captura" in low or "screenshot" in low or "ver pantalla" in low or "ver la pantalla" in low:
            try:
                from modules.humanoid.nerve import eyes_capture
                cap = eyes_capture(use_nexus_if_available=True, source="screen")
                result["action"] = "eyes_capture"
                result["details"] = cap
                result["ok"] = bool(cap.get("ok"))
                if cap.get("evidence_path"):
                    result["evidence_path"] = cap["evidence_path"]
                self._append_log(artifacts, f"EYES_CAPTURE ok={cap.get('ok')} path={cap.get('evidence_path')}")
                return result
            except Exception as e:
                result["action"] = "eyes_capture"
                result["details"] = {"ok": False, "error": str(e)}
                result["ok"] = False
                self._append_log(artifacts, "EYES_CAPTURE ERROR: " + str(e))
                return result

        # 3) Cámara (salud NEXUS) + evidencia
        if "cámara" in low or "camara" in low or "cameras" in low or "camera" in low:
            out = self._run_camera_health()
            result["action"] = "robot_camera_health"
            result["details"] = out
            result["ok"] = bool(out.get("ok"))
            self._append_log(artifacts, "CAMERA_HEALTH:\n" + json.dumps(out, ensure_ascii=False)[:4000])
            return result

        # 4) Ejecutar tests / diagnósticos por shell (real)
        if "pytest" in low or "tests" in low or "smoke" in low:
            cmd = "python -m pytest -q"
            out = self._run_shell(artifacts, cmd, timeout_s=180)
            result["action"] = "shell:pytest"
            result["details"] = out
            result["ok"] = bool(out.get("ok"))
            return result

        # 4b) ATLAS_ARCHITECT: crear app / scaffolding autónomo
        if ("diseña una app" in low) or ("disena una app" in low) or ("app de inventario" in low) or ("inventario" in low and "app" in low):
            try:
                from modules.humanoid.mode.config import get_system_mode
                from modules.atlas_architect import AtlasArchitect

                sys_mode = get_system_mode()
                gov_mode = "growth" if sys_mode in ("aggressive", "ultra") else "governed"
                arch = AtlasArchitect(repo_root=self.repo_root)
                out = arch.execute_order(desc, mode=gov_mode, prefer_free=True, max_heal_attempts=3)
                result["action"] = "architect:order"
                result["details"] = out
                result["ok"] = bool(out.get("ok"))
                self._append_log(artifacts, "ARCHITECT:\n" + json.dumps(out, ensure_ascii=False)[:6000])
                return result
            except Exception as e:
                result["action"] = "architect:order"
                result["details"] = {"ok": False, "error": str(e)}
                result["ok"] = False
                self._append_log(artifacts, "ARCHITECT ERROR: " + str(e))
                return result

        # 5) Acción no reconocida: NO marcar ok. Pide interacción humana.
        result["action"] = "needs_human"
        result["details"] = {
            "message": "Paso no mapeado a una acción real. Requiere que el Owner elija: (a) comando a ejecutar, (b) módulo/archivo a inspeccionar, (c) abrir Web/Pies o PC Walker.",
            "hint": "Usa modo=controlled para ejecutar paso a paso y ver terminal.",
        }
        result["ok"] = False
        self._append_log(artifacts, "NEEDS_HUMAN: " + desc)
        return result

    def execute_steps(self, steps: List[Dict[str, Any]], goal: str) -> Dict[str, Any]:
        artifacts = self.new_run_artifacts(goal)
        executed: List[Dict[str, Any]] = []
        evidence: List[str] = []

        self._append_log(artifacts, f"CURSOR RUN: {artifacts.run_id}")
        self._append_log(artifacts, f"GOAL: {goal}")
        self._append_log(artifacts, "")

        for idx, s in enumerate(steps):
            self._append_log(artifacts, f"\n=== STEP {idx + 1}/{len(steps)} ===")
            r = self.execute_step(artifacts, s)
            r["step_index"] = idx
            executed.append(r)
            if r.get("evidence_path"):
                evidence.append(str(r["evidence_path"]))
            # Estado del step para UI
            if r.get("ok"):
                s["status"] = "done"
            elif r.get("action") == "needs_human":
                s["status"] = "needs_human"
            else:
                s["status"] = "failed"

        ok_all = all(x.get("ok") for x in executed if x.get("action") != "needs_human") and any(x.get("ok") for x in executed)
        # Guardar JSON de run (para inspección humana)
        payload = {
            "run_id": artifacts.run_id,
            "goal": goal,
            "steps": steps,
            "executed": executed,
            "evidence": evidence,
            "terminal_log_path": str(artifacts.log_path),
            "script_path": str(artifacts.script_path),
            "ts_utc": datetime.now(timezone.utc).isoformat(),
        }
        try:
            _atomic_write(artifacts.json_path, json.dumps(payload, ensure_ascii=False, indent=2))
        except Exception:
            pass

        # Preview terminal (últimos N chars)
        preview = ""
        try:
            if artifacts.log_path.exists():
                preview = artifacts.log_path.read_text(encoding="utf-8", errors="ignore")[-6000:]
        except Exception:
            preview = ""

        # Incluir paths en evidencia para que la UI los muestre
        evidence_extended = list(evidence)
        evidence_extended.append(str(artifacts.script_path))
        evidence_extended.append(str(artifacts.log_path))
        evidence_extended.append(str(artifacts.json_path))

        return {
            "ok": ok_all,
            "executed": executed,
            "evidence": evidence_extended,
            "artifacts": {
                "run_id": artifacts.run_id,
                "run_dir": str(artifacts.run_dir),
                "script_path": str(artifacts.script_path),
                "terminal_log_path": str(artifacts.log_path),
                "run_json_path": str(artifacts.json_path),
            },
            "terminal_preview": preview or "—",
        }

