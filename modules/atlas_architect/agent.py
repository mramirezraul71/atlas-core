from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from .ans_logger import ANSBitacoraLogger
from .fs_tools import FilesystemTools
from .indexer import ArchitectureIndexer
from .project_indexer import ProjectIndexer
from .scaffolder import AppScaffolder
from .self_heal import SelfHealingLoop
from .governance import ChangePlan, apply_change_plan
from .venv_manager import VenvManager
from .model_orchestrator import MultiModelOrchestrator
from .terminal_tools import TerminalTools


@dataclass
class ArchitectState:
    arch_index_path: str = ""
    last_test: Optional[Dict[str, Any]] = None
    last_suggestion: Optional[str] = None


class AtlasArchitect:
    """ATLAS_ARCHITECT: agente agentic de codificación.

    - Indexa arquitectura (PUSH/NEXUS/ROBOT).
    - Provee herramientas FS.
    - Ejecuta terminal (pytest) y analiza errores.
    - Loggea a ANS (diff + justificación) cada cambio sugerido o realizado.
    - Orquesta modelos para edición pesada (si hay IA).
    """

    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or self._repo_root()).resolve()
        self.ans = ANSBitacoraLogger(self.repo_root)
        self.fs = FilesystemTools(self.repo_root, ans_logger=self.ans)
        self.term = TerminalTools(self.repo_root)
        self.models = MultiModelOrchestrator()
        self.indexer = ArchitectureIndexer(self.repo_root)
        self.project_indexer = ProjectIndexer(self.repo_root)
        self.scaffold = AppScaffolder(self.repo_root, self.fs, ans=self.ans)
        self.healer = SelfHealingLoop(self)
        self.venv = VenvManager(self.term, ans=self.ans)
        self.state = ArchitectState()

    def _repo_root(self) -> Path:
        root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
        if root:
            return Path(root).resolve()
        return Path(__file__).resolve().parents[2]

    def index_architecture(self) -> Dict[str, Any]:
        idx = self.indexer.build()
        path = self.indexer.persist(idx)
        self.state.arch_index_path = path
        self.ans.log_suggestion(f"Index de arquitectura generado: {path}")
        return {"ok": True, "index": idx.to_dict(), "path": path}

    def index_project(self, project_root: str, name: str = "project_index") -> Dict[str, Any]:
        """Indexa cualquier proyecto (ATLAS o apps nuevas) para entender arquitectura/deps antes de editar."""
        root = Path(project_root).resolve()
        idx = self.project_indexer.index(root)
        path = self.project_indexer.persist(idx, name=name)
        self.ans.log_suggestion(f"Index de proyecto generado: {path}")
        return {"ok": True, "index": idx.to_dict(), "path": path}

    def run_pytest_and_analyze(self, nodeid: Optional[str] = None, cwd: Optional[Path] = None) -> Dict[str, Any]:
        res = self.term.run_pytest(nodeid=nodeid, cwd=cwd)
        self.state.last_test = res
        if res.get("ok"):
            self.ans.log_suggestion("pytest OK. No se requiere refactor.")
            return {"ok": True, "result": res, "suggestion": None}

        issues = res.get("issues") or []
        summary = f"pytest FAIL. issues={len(issues)}. returncode={res.get('returncode')}"
        self.ans.log_suggestion(summary)

        # Sugerencia determinista: mostrar archivos más repetidos.
        top_file = None
        try:
            from collections import Counter
            c = Counter([i.get("file") for i in issues if i.get("file")])
            top_file = c.most_common(1)[0][0] if c else None
        except Exception:
            top_file = None

        suggestion = {
            "summary": summary,
            "top_file": top_file,
            "next_action": "Abrir el archivo top_file y corregir según traceback. Re-ejecutar pytest.",
        }
        self.state.last_suggestion = suggestion["summary"]
        return {"ok": False, "result": res, "suggestion": suggestion}

    def propose_code_fix(self, problem: str, context: Optional[Dict[str, Any]] = None, prefer_free: bool = True) -> Dict[str, Any]:
        """Genera propuesta de cambio (no aplica) usando orquestación multi-modelo."""
        ctx = context or {}
        arch = {}
        try:
            arch = self.indexer.build().to_dict()
        except Exception:
            arch = {}

        prompt = (
            "Eres ATLAS_ARCHITECT, un agente senior de codificación para ATLAS_PUSH.\n"
            "Responde en ESPAÑOL.\n"
            "Devuelve un JSON con: files_to_edit[], rationale, patch_plan (pasos), tests_to_run.\n\n"
            f"PROBLEMA:\n{problem}\n\n"
            f"ARQUITECTURA:\n{arch}\n\n"
            f"CONTEXTO EXTRA:\n{ctx}\n"
        )
        out = self.models.run(prompt, intent_hint="code", prefer_free=prefer_free, timeout_s=60)
        self.ans.log_suggestion(f"Propuesta generada (modelo={out.model_routing.get('model_key')}). Motivo: {out.model_routing.get('reason')}")
        return {"ok": out.ok, "proposal": out.text, "model_routing": out.model_routing}

    def execute_order(
        self,
        order: str,
        *,
        mode: str = "governed",
        prefer_free: bool = True,
        max_heal_attempts: int = 3,
    ) -> Dict[str, Any]:
        """Entrada principal: recibe una orden tipo 'Diseña una app...' y ejecuta flujo autónomo.

        - mode=governed: genera plan + diffs + encola aprobación (NO aplica).
        - mode=growth: aplica cambios + corre tests + self-heal.
        """
        text = (order or "").strip()
        low = text.lower()
        mode_norm = (mode or "governed").strip().lower()
        mode_norm = "growth" if mode_norm in ("growth", "aggressive", "auto") else "governed"

        self.ans.log_suggestion(f"Orden recibida: {text[:200]}")
        arch = self.index_architecture()
        repo_idx = self.index_project(str(self.repo_root), name="atlas_push_index")

        # Heurística inicial: apps de inventario (Rauli)
        is_inventory = ("inventario" in low) or ("inventory" in low)
        org_name = "rauli" if ("rauli" in low or "panader" in low) else "app"

        if is_inventory:
            if mode_norm == "governed":
                plan = self.scaffold.plan_inventory_app(org_name=org_name, app_name="inventario", mode="governed")
                try:
                    from modules.humanoid.approvals import create as approvals_create

                    payload = plan.to_payload()
                    payload["arch_index_path"] = arch.get("path")
                    payload["repo_index_path"] = repo_idx.get("path")
                    queued = approvals_create(action="architect_apply", payload=payload)
                    approval_id = queued.get("approval_id")
                except Exception as e:
                    approval_id = None
                    self.ans.log_debug(f"Fallo encolando aprobación: {e}", ok=False)

                self.ans.log_suggestion("[ARCHITECT] Diseñando arquitectura para nueva App... (GOVERNED)")
                return {
                    "ok": True,
                    "mode": "governed",
                    "approval_id": approval_id,
                    "plan": plan.to_payload(),
                    "arch": arch,
                    "repo_index": repo_idx,
                }

            # growth: aplicar scaffold y probar
            self.ans.log_suggestion("[ARCHITECT] Diseñando arquitectura para nueva App... (GROWTH)")
            sc = self.scaffold.scaffold_inventory_app(org_name=org_name, app_name="inventario")

            backend_dir = Path(sc.project_root) / "backend"
            # Aislamiento (Regla de Oro): crear venv en backend y ejecutar dentro
            vinfo = self.venv.ensure_venv(backend_dir, venv_name=".venv")
            deps_ok = False
            if vinfo.ok:
                deps_ok = self.venv.install_requirements(vinfo.python_exe, backend_dir / "requirements.txt", cwd=backend_dir)

            test_cmd = f'"{vinfo.python_exe}" -m pytest -q' if vinfo.ok else "python -m pytest -q"
            heal = self.healer.heal_command(
                test_cmd,
                max_attempts=int(max_heal_attempts or 3),
                prefer_free=prefer_free,
                governed=False,
                cwd=backend_dir,
            )
            r2 = self.term.run_command(test_cmd, timeout_s=300, cwd=backend_dir)

            return {
                "ok": bool(sc.ok) and bool(r2.get("ok")) and bool(vinfo.ok) and bool(deps_ok),
                "mode": "growth",
                "scaffold": sc.__dict__,
                "tests": r2,
                "self_heal": heal,
                "venv": vinfo.__dict__,
                "deps_ok": deps_ok,
                "arch": arch,
                "repo_index": repo_idx,
            }

        # fallback: si no se reconoce el tipo, sugerir plan con IA (sin aplicar)
        self.ans.log_suggestion("Tipo de app no reconocido por heurística; generando propuesta.")
        prop = self.propose_code_fix(f"Diseñar app desde orden: {text}", context={"arch": arch, "repo_index": repo_idx}, prefer_free=prefer_free)
        return {"ok": False, "mode": mode_norm, "order": text, "proposal": prop}

    def apply_approved_plan(self, approval_id: str) -> Dict[str, Any]:
        """Aplica un plan previamente aprobado en la cola de approvals."""
        try:
            from modules.humanoid.approvals import get as approvals_get
        except Exception as e:
            return {"ok": False, "error": f"approvals_unavailable: {e}"}

        item = approvals_get(approval_id)
        if not item:
            return {"ok": False, "error": "approval_not_found"}
        if (item.get("status") or "").lower() != "approved":
            return {"ok": False, "error": "approval_not_approved", "status": item.get("status")}

        payload = item.get("payload") or {}
        changes = payload.get("changes") or []
        plan = ChangePlan(goal=payload.get("goal") or "approved_plan", mode="growth", changes=[])
        from .governance import PlannedChange

        for ch in changes:
            plan.changes.append(
                PlannedChange(
                    kind=ch.get("kind") or "write_file",
                    path=ch.get("path") or "",
                    justification=ch.get("justification") or "",
                    content=ch.get("content"),
                    ops=ch.get("ops"),
                    diff_preview=ch.get("diff_preview") or "",
                )
            )
        applied = apply_change_plan(plan, self.fs)
        self.ans.log_suggestion(f"Plan aprobado aplicado: approval_id={approval_id} cambios={len(applied)}")
        return {"ok": True, "approval_id": approval_id, "applied": applied}

    def capture_and_ocr_screen(self) -> Dict[str, Any]:
        """OCR (Tesseract) de pantalla: captura → OCR. Útil si una UI falla y solo hay error visual."""
        try:
            from modules.humanoid.screen.capture import capture_screen, save_capture_to_file
            from modules.humanoid.screen.ocr import run_ocr

            img, err = capture_screen(region=None, format="png")
            if err or not img:
                self.ans.log_debug(f"OCR: fallo captura pantalla: {err}", ok=False)
                return {"ok": False, "error": err or "capture_failed", "text": "", "evidence_path": None}
            evidence_path = save_capture_to_file(img, dir_path=str(self.repo_root / "snapshots" / "architect"), prefix="architect_screen")
            text, oerr = run_ocr(image_bytes=img, image_path=None)
            ok = bool(text) and not oerr
            self.ans.log_debug(f"OCR ejecutado ok={ok} err={oerr}", ok=ok)
            return {"ok": ok, "error": oerr, "text": text, "evidence_path": evidence_path}
        except Exception as e:
            self.ans.log_debug(f"OCR exception: {e}", ok=False)
            return {"ok": False, "error": str(e), "text": "", "evidence_path": None}

    def agentic_execute(
        self,
        goal: str,
        *,
        prefer_free: bool = True,
        max_iters: int = 3,
    ) -> Dict[str, Any]:
        """Loop agentic: el modelo decide tool_calls (JSON) y el agente ejecuta.

        Herramientas disponibles (funciones):
        - read_file(path)
        - write_file(path, content, justification)
        - create_directory(path)
        - list_files(path=".", pattern="*", limit=200)
        - run_pytest(nodeid=None)
        - run_command(cmd, timeout_s=180)  (SAFE: allowlist en SafeShellExecutor)
        """
        self.index_architecture()
        transcript: List[Dict[str, Any]] = []
        tool_outputs: List[Dict[str, Any]] = []

        tools_spec = [
            {"name": "read_file", "args": {"path": "string"}},
            {"name": "write_file", "args": {"path": "string", "content": "string", "justification": "string"}},
            {"name": "atomic_patch_preview", "args": {"path": "string", "ops": "list"}},
            {"name": "atomic_patch_apply", "args": {"path": "string", "ops": "list", "justification": "string"}},
            {"name": "create_directory", "args": {"path": "string"}},
            {"name": "list_files", "args": {"path": "string", "pattern": "string", "limit": "int"}},
            {"name": "run_pytest", "args": {"nodeid": "string|null"}},
            {"name": "run_command", "args": {"cmd": "string", "timeout_s": "int"}},
            {"name": "ocr_screen", "args": {}},
        ]

        def _extract_json(text: str) -> Dict[str, Any]:
            # Extract first {...} block if present
            t = (text or "").strip()
            if not t:
                return {}
            try:
                return json.loads(t)
            except Exception:
                pass
            import re as _re

            m = _re.search(r"(\{[\s\S]*\})", t)
            if m:
                try:
                    return json.loads(m.group(1))
                except Exception:
                    return {}
            return {}

        for it in range(max(1, min(8, int(max_iters or 3)))):
            prompt = (
                "Eres ATLAS_ARCHITECT (agente de codificación). Idioma: ESPAÑOL.\n"
                "Tu objetivo: resolver el goal con acciones REALES (FS + terminal) y dejar evidencia.\n"
                "Devuelve SOLO JSON válido con estas claves:\n"
                "- plan (lista corta)\n"
                "- tool_calls (lista) cada uno: {name, args}\n"
                "- rationale (1-3 líneas)\n"
                "- done (bool)\n\n"
                f"TOOLS:\n{json.dumps(tools_spec, ensure_ascii=False)}\n\n"
                f"GOAL:\n{goal}\n\n"
                f"ÚLTIMOS TOOL OUTPUTS (resumen):\n{json.dumps(tool_outputs[-6:], ensure_ascii=False) if tool_outputs else '[]'}\n"
            )
            model = self.models.run(prompt, intent_hint="architect", prefer_free=prefer_free, timeout_s=60)
            parsed = _extract_json(model.text)
            transcript.append({"iter": it + 1, "model_routing": model.model_routing, "raw": model.text[:4000], "json": parsed})
            self.ans.log_suggestion(f"ATLAS_ARCHITECT iter={it+1}: plan recibido. done={bool(parsed.get('done'))}")

            calls = parsed.get("tool_calls") or []
            if not isinstance(calls, list):
                calls = []

            for call in calls[:10]:
                name = (call.get("name") or "").strip()
                args = call.get("args") or {}
                out: Dict[str, Any] = {"tool": name, "ok": False, "args": args}
                try:
                    if name == "read_file":
                        out["result"] = self.fs.read_file(args.get("path") or "")
                        out["ok"] = True
                    elif name == "write_file":
                        rec = self.fs.write_file(
                            args.get("path") or "",
                            args.get("content") or "",
                            justification=args.get("justification") or "atlas_architect change",
                        )
                        out["result"] = {"file_path": rec.file_path, "diff_path": rec.diff_path, "justification": rec.justification}
                        out["ok"] = True
                    elif name == "atomic_patch_preview":
                        # ops: lista de dicts serializables {type: replace_block|regex_replace|insert_after, ...}
                        from .governance import deserialize_patch_ops

                        ops = deserialize_patch_ops(args.get("ops") or [])
                        out["result"] = self.fs.atomic_patch_preview(args.get("path") or "", ops)
                        out["ok"] = True
                    elif name == "atomic_patch_apply":
                        from .governance import deserialize_patch_ops

                        ops = deserialize_patch_ops(args.get("ops") or [])
                        rec = self.fs.atomic_patch(
                            args.get("path") or "",
                            ops,
                            justification=args.get("justification") or "atomic patch (agentic)",
                        )
                        out["result"] = {"file_path": rec.file_path, "diff_path": rec.diff_path, "justification": rec.justification}
                        out["ok"] = True
                    elif name == "create_directory":
                        r = self.fs.create_directory(args.get("path") or "")
                        out["result"] = {"path": r.path, "error": r.error}
                        out["ok"] = r.ok
                    elif name == "list_files":
                        out["result"] = self.fs.list_files(
                            path=args.get("path") or ".",
                            pattern=args.get("pattern") or "*",
                            limit=int(args.get("limit") or 200),
                        )
                        out["ok"] = True
                    elif name == "run_pytest":
                        out["result"] = self.term.run_pytest(nodeid=args.get("nodeid"))
                        out["ok"] = bool(out["result"].get("ok"))
                    elif name == "run_command":
                        out["result"] = self.term.run(str(args.get("cmd") or ""), timeout_s=int(args.get("timeout_s") or 180)).__dict__
                        out["ok"] = bool(out["result"].get("ok"))
                    elif name == "ocr_screen":
                        out["result"] = self.capture_and_ocr_screen()
                        out["ok"] = bool(out["result"].get("ok"))
                    else:
                        out["result"] = {"error": "unknown_tool"}
                        out["ok"] = False
                except Exception as e:
                    out["error"] = str(e)
                    out["ok"] = False

                tool_outputs.append(out)
                # Log técnico (Bitácora)
                self.ans.log_suggestion(f"ATLAS_ARCHITECT tool={name} ok={out.get('ok')}")

            if bool(parsed.get("done")):
                break

        return {"ok": True, "transcript": transcript, "tool_outputs": tool_outputs[-50:]}

