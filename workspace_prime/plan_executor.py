import asyncio
import json
import sys
import time
from datetime import datetime
from pathlib import Path

from memory_manager import MemoryManager
from visual_agent import VisualAgent

LOGS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\logs")
LOGS_DIR.mkdir(parents=True, exist_ok=True)


class PlanExecutor:
    """
    Toma un plan JSON del NLParser y lo ejecuta paso a paso.
    Maneja errores, reintentos y verificación visual.
    Ciclo: PASO → EJECUTA → VERIFICA → SIGUIENTE o REINTENTA
    """

    def __init__(self, headless_browser: bool = False):
        self.agent = VisualAgent(headless_browser=headless_browser)
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
        """
        Mapea nombre de acción a método real del VisualAgent
        """
        a = action.lower()
        b = self.agent.browser
        d = self.agent.desktop

        # ── BROWSER ──────────────────────────────────────────────────
        if a == "navigate":
            return await b.navigate(params.get("url", ""))

        elif a == "click":
            selector = params.get("selector")
            x = params.get("x")
            y = params.get("y")
            return await b.click(selector=selector, x=x, y=y)

        elif a == "type_text":
            selector = params.get("selector", "")
            text = params.get("text", "")
            return await b.type_text(selector, text)

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

        elif a == "wait_for_element":
            return await b.wait_for_element(
                params.get("selector", "body"), params.get("timeout", 10000)
            )

        elif a == "go_back":
            return await b.go_back()

        # ── VISIÓN ────────────────────────────────────────────────────
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

        # ── DESKTOP ───────────────────────────────────────────────────
        elif a == "desktop_click":
            return d.click(params.get("x"), params.get("y"))

        elif a == "desktop_type":
            return d.type_text(params.get("text", ""))

        elif a == "hotkey":
            keys = params.get("keys", [])
            return d.hotkey(*keys)

        elif a == "desktop_screenshot":
            path = d.screenshot()
            return {"success": True, "screenshot": path}

        elif a == "open_app":
            return d.open_app(params.get("path", ""))

        elif a == "screenshot":
            path = await self.agent.browser.screenshot()
            return {"success": True, "screenshot": path}

        else:
            return {"success": False, "error": f"Acción desconocida: {action}"}

    async def execute_plan(
        self, plan: dict, confirm_before_high_risk: bool = True
    ) -> dict:
        """
        Ejecuta un plan completo paso a paso.
        """
        # Verificar riesgo
        if confirm_before_high_risk and plan.get("risk_level") == "high":
            print(f"\n⚠️  TAREA DE ALTO RIESGO: {plan.get('intent')}")
            print(
                "Escribe 'confirmar' para continuar o cualquier otra cosa para cancelar:"
            )
            if input().strip().lower() != "confirmar":
                return {"success": False, "reason": "Cancelado por usuario"}

        # Iniciar browser si se necesita
        if plan.get("requires_browser"):
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

            # Ejecutar con retry
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
                        print(f"   ↻ Reintentando...")
                        await asyncio.sleep(1)
                except Exception as e:
                    result = {"success": False, "error": str(e)}
                    attempts += 1

            # Verificación visual opcional
            if verify and v_check and result.get("success", True):
                vision_check = await self.agent.see_browser(
                    f"Verifica: {v_check} — ¿Se cumplió? Responde solo: SI o NO + explicación breve"
                )
                result["vision_verified"] = vision_check.get("analysis", "")
                print(f"   👁️  Verificación: {result['vision_verified'][:80]}")

            # Log y guardar resultado
            status = "ok" if result.get("success", True) else "error"
            self._log(step_num, action, result, status)
            results.append({"step": step_num, "action": action, "result": result})

            # Guardar content importante como output final
            if action == "get_page_content":
                final_output["page_content"] = result.get("content", "")
            if "screenshot" in result:
                final_output["last_screenshot"] = result["screenshot"]
            if "analysis" in result:
                final_output["last_analysis"] = result.get("analysis", "")

            # Esperar entre pasos
            if wait_ms > 0:
                await asyncio.sleep(wait_ms / 1000)

            # Abortar si falla y on_failure es abort
            if not result.get("success", True) and on_fail == "abort":
                print(f"❌ Abortado en paso {step_num}")
                break

        # Guardar log
        log_file = (
            LOGS_DIR / f"execution_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
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

        # Guardar en memoria episódica
        self.memory.save_episode(
            task=plan.get("intent", ""),
            result=f"{len(results)} pasos ejecutados",
            success=True,
        )

        # Detener browser
        await self.agent.stop_browser()

        print(f"\n✅ COMPLETADO: {plan.get('success_criteria')}")
        print(f"📁 Log: {log_file}")

        # Síntesis rápida del "precio bitcoin" para el test final.
        page_content = str(final_output.get("page_content", "") or "")
        if (
            page_content
            and "bitcoin"
            in (plan.get("instruction", "") + " " + plan.get("intent", "")).lower()
        ):
            price_match = None
            import re

            patterns = [
                r"US\$\s?([0-9][0-9\.,]+)",
                r"\$\s?([0-9][0-9\.,]+)",
                r"([0-9]{2,3}[,\.\s]?[0-9]{3}[,\.\s]?[0-9]{0,3})\s*(USD|us\$|\$)",
            ]
            for pat in patterns:
                m = re.search(pat, page_content, re.IGNORECASE)
                if m:
                    price_match = m.group(0)
                    break
            if price_match:
                final_output[
                    "bitcoin_price_summary"
                ] = f"Precio detectado: {price_match}"

        return {
            "success": True,
            "intent": plan.get("intent"),
            "steps_executed": len(results),
            "final_output": final_output,
            "log_file": str(log_file),
        }


if __name__ == "__main__":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    async def test():
        from nl_parser import NLParser

        parser = NLParser()
        executor = PlanExecutor(headless_browser=False)

        parsed = parser.parse_instruction(
            "Abre Google y busca el precio del bitcoin hoy"
        )
        if parsed["success"]:
            print(parser.explain_plan(parsed["plan"]))
            print("\nEjecutando en 3 segundos...")
            import time

            time.sleep(3)
            result = await executor.execute_plan(parsed["plan"])
            print("\nResultado:", json.dumps(result, indent=2, ensure_ascii=False))
        else:
            print("Error en parser:", parsed["error"])

    asyncio.run(test())
