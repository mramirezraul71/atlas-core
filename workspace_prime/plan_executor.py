import asyncio
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from memory_manager import MemoryManager
from smart_browser import SmartBrowser
from visual_agent import VisualAgent

ATLAS_ROOT = Path(r"C:\ATLAS_PUSH").resolve()
LOGS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\logs")
LOGS_DIR.mkdir(parents=True, exist_ok=True)


class PlanExecutor:
    """
    Ejecuta planes JSON paso a paso con feedback semántico.
    """

    def __init__(self, headless_browser: bool = False):
        self.agent = VisualAgent(headless_browser=headless_browser)
        self.smart = SmartBrowser()
        self.memory = MemoryManager()
        self.execution_log = []

    def _log(self, step: int, action: str, result: dict, status: str):
        entry = {
            "timestamp": datetime.now().isoformat(),
            "step": step,
            "action": action,
            "result_summary": str(result)[:300],
            "status": status,
        }
        self.execution_log.append(entry)
        icon = "✅" if status == "ok" else "❌" if status == "error" else "⚠️"
        print(f"{icon} Paso {step}: {action} → {status}")

    def _resolve_path(self, raw_path: str) -> Path:
        path = Path(raw_path)
        resolved = path.resolve() if path.is_absolute() else (ATLAS_ROOT / path).resolve()
        if ATLAS_ROOT not in resolved.parents and resolved != ATLAS_ROOT:
            raise ValueError(f"Ruta fuera del workspace: {resolved}")
        return resolved

    def _run_command(self, command: str, timeout: int = 60, cwd: str | None = None) -> dict:
        working_dir = self._resolve_path(cwd) if cwd else ATLAS_ROOT
        proc = subprocess.run(
            ["powershell", "-Command", command],
            cwd=str(working_dir),
            capture_output=True,
            text=True,
            timeout=min(int(timeout or 60), 300),
            encoding="utf-8",
            errors="replace",
        )
        return {
            "success": proc.returncode == 0,
            "returncode": proc.returncode,
            "stdout": proc.stdout[-4000:],
            "stderr": proc.stderr[-4000:],
            "cwd": str(working_dir),
        }

    def _collect_blockers(self, final_output: dict, results: list) -> list[str]:
        blob_parts = [json.dumps(final_output or {}, ensure_ascii=False).lower()]
        for item in results:
            blob_parts.append(json.dumps(item.get("result", {}), ensure_ascii=False).lower())
        blob = " ".join(blob_parts)
        blockers = []
        if "captcha" in blob or "acerca de esta página" in blob or "/sorry/" in blob:
            blockers.append("web_antibot_detected")
        if "acción desconocida" in blob or "accion desconocida" in blob:
            blockers.append("unsupported_action")
        if "error" in blob and "success\": false" in blob:
            blockers.append("step_errors_present")
        return blockers

    def _build_summary(self, plan: dict, results: list, blockers: list[str]) -> str:
        ok_steps = sum(1 for item in results if item.get("result", {}).get("success", True))
        total_steps = len(results)
        if blockers:
            return (
                f"Plan ejecutado con bloqueos ({', '.join(blockers)}). "
                f"Pasos OK: {ok_steps}/{total_steps}."
            )
        return f"Plan ejecutado correctamente. Pasos OK: {ok_steps}/{total_steps}."

    async def execute_action(self, action: str, params: dict) -> dict:
        a = action.lower()
        b = self.agent.browser
        d = self.agent.desktop

        if a == "smart_browser_task":
            return await self.smart.run_task(params.get("task", ""))
        if a == "navigate":
            return await b.navigate(params.get("url", ""))
        if a == "click":
            return await b.click(
                selector=params.get("selector"), x=params.get("x"), y=params.get("y")
            )
        if a == "type_text":
            return await b.type_text(params.get("selector", ""), params.get("text", ""))
        if a == "press_key":
            return await b.press_key(params.get("key", "Enter"))
        if a == "scroll":
            return await b.scroll(
                params.get("direction", "down"), params.get("amount", 300)
            )
        if a == "get_page_content":
            return await b.get_page_content()
        if a == "new_tab":
            return await b.new_tab(params.get("url"))
        if a == "execute_js":
            return await b.execute_js(params.get("script", ""))
        if a == "go_back":
            return await b.go_back()
        if a == "wait_for_element":
            return await b.wait_for_element(
                params.get("selector", "body"), params.get("timeout", 10000)
            )
        if a == "screenshot":
            path = await b.screenshot()
            return {"success": True, "screenshot": path}
        if a == "see_browser":
            return await self.agent.see_browser(params.get("question", "¿Qué ves?"))
        if a == "find_and_click_visual":
            return await self.agent.find_and_click_visual(
                params.get("element_description", "")
            )
        if a == "see_desktop":
            return self.agent.see_desktop(params.get("question", "¿Qué ves?"))
        if a == "navigate_and_analyze":
            return await self.agent.navigate_and_analyze(params.get("url", ""))
        if a == "desktop_click":
            return d.click(params.get("x"), params.get("y"))
        if a == "desktop_type":
            return d.type_text(params.get("text", ""))
        if a == "hotkey":
            return d.hotkey(*params.get("keys", []))
        if a == "desktop_screenshot":
            path = d.screenshot()
            return {"success": True, "screenshot": path}
        if a == "open_app":
            return d.open_app(params.get("path", ""))
        if a == "read_file":
            path = self._resolve_path(params.get("path", ""))
            return {"success": True, "path": str(path), "content": path.read_text(encoding="utf-8", errors="replace")[:6000]}
        if a == "write_file":
            path = self._resolve_path(params.get("path", ""))
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(str(params.get("content", "")), encoding="utf-8")
            return {"success": True, "path": str(path), "bytes_written": len(str(params.get("content", "")))}
        if a == "execute_command":
            return self._run_command(
                params.get("command", ""),
                timeout=params.get("timeout", 60),
                cwd=params.get("working_directory"),
            )
        if a == "memory_read":
            key = params.get("key")
            working = self.memory.read_working()
            longterm = self.memory.read_longterm()
            if key:
                return {
                    "success": True,
                    "key": key,
                    "working_value": working.get(key),
                    "longterm_rules": [
                        rule for rule in longterm.get("rules", []) if key in rule.get("keywords", [])
                    ][:5],
                }
            return {"success": True, "working": working, "longterm": longterm}
        if a == "memory_write":
            key = params.get("key")
            if not key:
                return {"success": False, "error": "memory_write requiere key"}
            self.memory.update_field(str(key), params.get("value"))
            return {"success": True, "key": key, "value": params.get("value")}
        return {"success": False, "error": f"Acción desconocida: {action}"}

    async def execute_plan(self, plan: dict, confirm_high_risk: bool = True) -> dict:
        if confirm_high_risk and plan.get("risk_level") == "high":
            if sys.stdin.isatty():
                print(f"\n[ALTO RIESGO] {plan.get('intent')}")
                print("Escribe 'confirmar' para continuar: ", end="")
                if input().strip().lower() != "confirmar":
                    return {"success": False, "reason": "Cancelado"}

        browser_started = False
        if plan.get("requires_browser") or plan.get("requires_vision"):
            await self.agent.start_browser()
            browser_started = True

        steps = plan.get("steps", [])
        results = []
        final_output = {}
        critical_failure = False
        errors = []

        print(f"\n🚀 EJECUTANDO: {plan.get('intent')}")
        print(f"📋 {len(steps)} pasos\n")

        try:
            for step in steps:
                step_num = step["step_number"]
                action = step["action"]
                params = step.get("params", {})
                wait_ms = step.get("wait_after_ms", 500)
                on_fail = step.get("on_failure", "retry")
                verify = step.get("verify_with_vision", False)
                v_check = step.get("vision_check", "")

                print(f"▶ Paso {step_num}/{len(steps)}: {step['description']}")

                result = None
                attempts = 0
                max_attempts = 2 if on_fail == "retry" else 1

                while attempts < max_attempts:
                    try:
                        result = await self.execute_action(action, params)
                        if result.get("success", True):
                            break
                        attempts += 1
                        if attempts < max_attempts:
                            print("   ↻ Reintentando...")
                            await asyncio.sleep(1)
                    except Exception as e:
                        result = {"success": False, "error": str(e)}
                        attempts += 1

                if verify and v_check and result.get("success", True) and browser_started:
                    vc = await self.agent.see_browser(
                        f"Verifica: {v_check} — ¿Se cumplió? Solo responde: SI o NO + explicación breve"
                    )
                    result["vision_verified"] = vc.get("analysis", "")
                    print(f"   👁️  Verificación: {result['vision_verified'][:80]}")

                status = "ok" if result.get("success", True) else "error"
                self._log(step_num, action, result, status)
                results.append({"step": step_num, "action": action, "result": result})

                if action == "get_page_content":
                    final_output["page_content"] = result.get("content", "")
                if "screenshot" in result:
                    final_output["last_screenshot"] = result["screenshot"]
                if "analysis" in result:
                    final_output["last_analysis"] = result.get("analysis", "")
                if "result" in result and action == "smart_browser_task":
                    final_output["smart_result"] = result.get("result", "")
                if not result.get("success", True):
                    critical_failure = True
                    errors.append(result.get("error") or f"Fallo en {action}")

                if wait_ms > 0:
                    await asyncio.sleep(wait_ms / 1000)

                if not result.get("success", True) and on_fail == "abort":
                    print(f"❌ Abortado en paso {step_num}")
                    break
        finally:
            if browser_started:
                await self.agent.stop_browser()

        blockers = self._collect_blockers(final_output, results)
        final_success = bool(results) and not critical_failure and not blockers
        summary = self._build_summary(plan, results, blockers)

        log_file = LOGS_DIR / f"exec_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        log_payload = {
            "plan": plan,
            "results": results,
            "log": self.execution_log,
            "final_output": final_output,
            "success": final_success,
            "summary": summary,
            "blockers": blockers,
            "errors": errors,
        }
        log_file.write_text(
            json.dumps(log_payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )

        self.memory.save_episode(
            task=plan.get("intent", ""),
            result=summary,
            success=final_success,
            metadata={
                "steps_executed": len(results),
                "blockers": blockers,
                "errors": errors[:5],
                "log_file": str(log_file),
            },
        )
        self.memory.learn_from_execution(
            task=plan.get("intent", ""),
            success=final_success,
            plan=plan,
            final_output=final_output,
            errors=errors,
            metadata={"log_file": str(log_file)},
        )

        print(f"\n[{'COMPLETADO' if final_success else 'INCOMPLETO'}] {summary}")
        print(f"📁 Log guardado: {log_file}")

        return {
            "success": final_success,
            "intent": plan.get("intent"),
            "steps_executed": len(results),
            "final_output": final_output,
            "log_file": str(log_file),
            "summary": summary,
            "blockers": blockers,
            "errors": errors[:5],
        }


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    print("OK PlanExecutor")
