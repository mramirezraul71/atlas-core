from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class HealAttempt:
    attempt: int
    ok: bool
    cmd: str
    returncode: int
    issues: List[Dict[str, Any]]
    stdout_tail: str
    stderr_tail: str
    applied_changes: List[Dict[str, Any]]


class SelfHealingLoop:
    """Terminal feedback loop: ejecutar→diagnosticar→corregir→reintentar."""

    def __init__(self, architect: "AtlasArchitect") -> None:  # noqa: F821
        self.arch = architect

    def heal_command(
        self,
        cmd: str,
        *,
        max_attempts: int = 3,
        prefer_free: bool = True,
        governed: bool = False,
        cwd: Optional["Path"] = None,  # noqa: F821
    ) -> Dict[str, Any]:
        attempts: List[HealAttempt] = []
        for i in range(1, max(1, int(max_attempts or 3)) + 1):
            self.arch.ans.log_debug(f"Ejecutando comando (attempt={i}): {cmd}", ok=True)
            r = self.arch.term.run_command(cmd, timeout_s=300, cwd=cwd)
            ok = bool(r.get("ok"))
            issues = r.get("issues") or []
            applied: List[Dict[str, Any]] = []

            if ok:
                self.arch.ans.log_debug("Ejecución OK. Self-healing finaliza.", ok=True)
                attempts.append(
                    HealAttempt(
                        attempt=i,
                        ok=True,
                        cmd=cmd,
                        returncode=int(r.get("returncode") or 0),
                        issues=issues,
                        stdout_tail=(r.get("stdout") or "")[-8000:],
                        stderr_tail=(r.get("stderr") or "")[-8000:],
                        applied_changes=applied,
                    )
                )
                return {"ok": True, "attempts": [a.__dict__ for a in attempts], "final": r}

            # Diagnóstico + propuesta (si hay modelo)
            self.arch.ans.log_debug("Error detectado en ejecución; iniciando proceso de auto-reparación...", ok=False)
            ocr = {}
            try:
                ocr = self.arch.capture_and_ocr_screen()
            except Exception:
                ocr = {}
            problem = (
                f"Comando falló: {cmd}\n\n"
                f"STDOUT:\n{(r.get('stdout') or '')[-12000:]}\n\n"
                f"STDERR:\n{(r.get('stderr') or '')[-12000:]}\n\n"
                f"OCR_SCREEN_TEXT:\n{(ocr.get('text') or '')[:4000]}\n"
            )
            proposal = self.arch.propose_code_fix(problem, context={"issues": issues}, prefer_free=prefer_free)

            # En modo governed: NO aplicar. Solo reportar y cortar.
            if governed:
                attempts.append(
                    HealAttempt(
                        attempt=i,
                        ok=False,
                        cmd=cmd,
                        returncode=int(r.get("returncode") or -1),
                        issues=issues,
                        stdout_tail=(r.get("stdout") or "")[-8000:],
                        stderr_tail=(r.get("stderr") or "")[-8000:],
                        applied_changes=[],
                    )
                )
                return {
                    "ok": False,
                    "attempts": [a.__dict__ for a in attempts],
                    "final": r,
                    "proposal": proposal,
                    "error": "governed_mode_requires_manual_apply",
                }

            # En growth: si el modelo devolvió tool_calls ejecutables, usar agentic_execute como “motor”.
            # Nota: si no hay IA disponible, agentic_execute no logrará parches automáticamente (NullProvider).
            if proposal.get("proposal"):
                # Pedir explícitamente tool_calls basados en el fallo.
                goal = f"Auto-fix del fallo del comando. Aplica parches atómicos. Luego re-ejecuta: {cmd}"
                out = self.arch.agentic_execute(goal, prefer_free=prefer_free, max_iters=2)
                applied.append({"agentic": out})

            attempts.append(
                HealAttempt(
                    attempt=i,
                    ok=False,
                    cmd=cmd,
                    returncode=int(r.get("returncode") or -1),
                    issues=issues,
                    stdout_tail=(r.get("stdout") or "")[-8000:],
                    stderr_tail=(r.get("stderr") or "")[-8000:],
                    applied_changes=applied,
                )
            )

        return {"ok": False, "attempts": [a.__dict__ for a in attempts], "error": "max_attempts_exceeded"}

