import asyncio
import json
import sys
from datetime import datetime
from pathlib import Path

from memory_manager import MemoryManager
from smart_browser import SmartBrowser
from visual_agent import VisualAgent

LOGS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\logs")
LOGS_DIR.mkdir(parents=True, exist_ok=True)


class PlanExecutor:
    """
    Ejecuta planes JSON paso a paso.
    Elige automáticamente entre BrowserHands (simple)
    o SmartBrowser (complejo con IA) según el plan.
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
            "result_summary": str(result)[:200],
            "status": status,
        }
        self.execution_log.append(entry)
        icon = "✅" if status == "ok" else "❌" if status == "error" else "⚠️"
        print(f"{icon} Paso {step}: {action} → {status}")

    async def execute_action(self, action: str, params: dict) -> dict:
        a = action.lower()
        b = self.agent.browser
        d = self.agent.desktop

        # ── SMART BROWSER (browser-use con IA) ──────────────────────
        if a == "smart_browser_task":
            return await self.smart.run_task(params.get("task", ""))

        # ── BROWSER SIMPLE (Playwright) ──────────────────────────────
        elif a == "navigate":
            return await b.navigate(params.get("url", ""))
        elif a == "click":
            return await b.click(
                selector=params.get("selector"), x=params.get("x"), y=params.get("y")
            )
        elif a == "type_text":
            return await b.type_text(params.get("selector", ""), params.get("text", ""))
        elif a == "press_key":
            return await b.press_key(params.get("key", "Enter"))
        elif a == "scroll":
            return await b.scroll(
                params.get("direction", "down"), params.get("amount", 300)
            )
        elif a == "get_page_content":
            return await b.get_page_content()
        elif a == "new_tab":
            return await b.new_tab(params.get("url"))
        elif a == "execute_js":
            return await b.execute_js(params.get("script", ""))
        elif a == "go_back":
            return await b.go_back()
        elif a == "wait_for_element":
            return await b.wait_for_element(
                params.get("selector", "body"), params.get("timeout", 10000)
            )
        elif a == "screenshot":
            path = await b.screenshot()
            return {"success": True, "screenshot": path}

        # ── VISIÓN ───────────────────────────────────────────────────
        elif a == "see_browser":
            return await self.agent.see_browser(params.get("question", "¿Qué ves?"))
        elif a == "find_and_click_visual":
            return await self.agent.find_and_click_visual(
                params.get("element_description", "")
            )
        elif a == "see_desktop":
            return self.agent.see_desktop(params.get("question", "¿Qué ves?"))
        elif a == "navigate_and_analyze":
            return await self.agent.navigate_and_analyze(params.get("url", ""))

        # ── DESKTOP ──────────────────────────────────────────────────
        elif a == "desktop_click":
            return d.click(params.get("x"), params.get("y"))
        elif a == "desktop_type":
            return d.type_text(params.get("text", ""))
        elif a == "hotkey":
            return d.hotkey(*params.get("keys", []))
        elif a == "desktop_screenshot":
            path = d.screenshot()
            return {"success": True, "screenshot": path}
        elif a == "open_app":
            return d.open_app(params.get("path", ""))

        else:
            return {"success": False, "error": f"Acción desconocida: {action}"}

    async def execute_plan(self, plan: dict, confirm_high_risk: bool = True) -> dict:
        if confirm_high_risk and plan.get("risk_level") == "high":
            if sys.stdin.isatty():
                print(f"\n[ALTO RIESGO] {plan.get('intent')}")
                print("Escribe 'confirmar' para continuar: ", end="")
                if input().strip().lower() != "confirmar":
                    return {"success": False, "reason": "Cancelado"}
            # Sin TTY: continuar sin preguntar

        # Iniciar browser si se necesita
        if plan.get("requires_browser") or plan.get("requires_vision"):
            await self.agent.start_browser()

        steps = plan.get("steps", [])
        results = []
        final_output = {}

        print(f"\n🚀 EJECUTANDO: {plan.get('intent')}")
        print(f"📋 {len(steps)} pasos\n")

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

            if verify and v_check and result.get("success", True):
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

            if wait_ms > 0:
                await asyncio.sleep(wait_ms / 1000)

            if not result.get("success", True) and on_fail == "abort":
                print(f"❌ Abortado en paso {step_num}")
                break

        log_file = LOGS_DIR / f"exec_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(log_file, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "plan": plan,
                    "results": results,
                    "log": self.execution_log,
                    "final_output": final_output,
                },
                f,
                indent=2,
                ensure_ascii=False,
            )

        self.memory.save_episode(
            task=plan.get("intent", ""),
            result=f"{len(results)} pasos ejecutados",
            success=True,
        )

        await self.agent.stop_browser()

        print(f"\n[COMPLETADO] {plan.get('success_criteria')}")
        print(f"📁 Log guardado: {log_file}")

        return {
            "success": True,
            "intent": plan.get("intent"),
            "steps_executed": len(results),
            "final_output": final_output,
            "log_file": str(log_file),
        }


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    print("OK PlanExecutor")
