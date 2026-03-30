import json
import re
from datetime import datetime

import load_credentials  # noqa: F401 — carga CREDENTIALS_FILE y .env
from llm_router import text_completion
from memory_manager import MemoryManager

SUPPORTED_ACTIONS = {
    "navigate",
    "click",
    "type_text",
    "press_key",
    "scroll",
    "get_page_content",
    "execute_js",
    "go_back",
    "new_tab",
    "wait_for_element",
    "screenshot",
    "smart_browser_task",
    "see_browser",
    "see_desktop",
    "find_and_click_visual",
    "navigate_and_analyze",
    "desktop_click",
    "desktop_type",
    "hotkey",
    "desktop_screenshot",
    "open_app",
    "read_file",
    "write_file",
    "execute_command",
    "memory_read",
    "memory_write",
}

ACTION_ALIASES = {
    "browser_screenshot": "screenshot",
    "see_page": "see_browser",
    "memory_save": "memory_write",
}

ACTION_CATALOG = """
Capacidades disponibles:
BROWSER_SIMPLE: navigate(url), click(selector|x,y), type_text(selector,text),
                press_key(key), scroll(direction,amount), get_page_content(),
                execute_js(script), go_back(), new_tab(url), wait_for_element(selector,timeout),
                screenshot()
BROWSER_SMART:  smart_browser_task(task) — para sitios complejos, login, anti-bot o flujos largos
DESKTOP:        desktop_click(x,y), desktop_type(text), hotkey(keys[]),
                desktop_screenshot(), open_app(path)
VISION:         see_browser(question), see_desktop(question),
                find_and_click_visual(element_description), navigate_and_analyze(url)
FILES:          read_file(path), write_file(path,content)
SHELL:          execute_command(command, timeout, working_directory)
MEMORY:         memory_read(key), memory_write(key,value)
""".strip()


class NLParser:
    """Convierte instrucciones en lenguaje natural a planes ejecutables."""

    def __init__(self):
        self.memory = MemoryManager()

    def parse_instruction(self, instruction: str, screen_context: str = "") -> dict:
        system = f"""
Eres el planificador de ATLAS-WORKSPACE-PRIME.
Conviertes instrucciones humanas en planes de ejecución JSON.

Solo puedes usar acciones de este catálogo y no debes inventar nombres nuevos:
{ACTION_CATALOG}

Reglas:
- Usa `smart_browser_task` si el sitio es complejo, hay login, muchas decisiones, SPA o riesgo de anti-bot.
- Usa `desktop_screenshot`, no `screenshot`, cuando la accion sea de escritorio.
- Si una tarea puede resolverse con 1 paso seguro, evita planes largos.
- Si la tarea es solo consultar memoria, usa `memory_read`.
- Responde SOLO con JSON válido sin texto adicional.

Formato:
{{
  "instruction": "instrucción original",
  "intent": "qué hace en una frase",
  "requires_browser": true,
  "requires_smart_browser": false,
  "requires_desktop": false,
  "requires_vision": false,
  "steps": [
    {{
      "step_number": 1,
      "description": "descripción en español",
      "action": "nombre_metodo",
      "params": {{}},
      "wait_after_ms": 1000,
      "verify_with_vision": false,
      "vision_check": "",
      "on_failure": "retry"
    }}
  ],
  "success_criteria": "cómo sé que terminó bien",
  "output_type": "screenshot|text|file|summary|json",
  "risk_level": "low|medium|high",
  "requires_confirmation": false
}}
"""
        memory_context = self.memory.get_relevant_context(instruction)
        user_parts = [f"Instrucción: {instruction}"]
        if screen_context:
            user_parts.append(f"Contexto visual actual:\n{screen_context}")
        if memory_context.get("recent_episodes"):
            recent_lines = []
            for ep in memory_context["recent_episodes"]:
                recent_lines.append(
                    f"- task={ep.get('task','')} | success={ep.get('success')} | result={str(ep.get('result',''))[:180]}"
                )
            user_parts.append("Episodios recientes relevantes:\n" + "\n".join(recent_lines))
        if memory_context.get("rules"):
            rule_lines = []
            for rule in memory_context["rules"]:
                rule_lines.append(f"- {rule.get('recommendation','')}")
            user_parts.append("Reglas aprendidas relevantes:\n" + "\n".join(rule_lines))

        out = text_completion("\n\n".join(user_parts), system=system, max_tokens=2048)
        if not out.get("success"):
            return {"success": False, "error": out.get("error", "Unknown error")}
        raw = (out.get("text") or "").strip()
        plan_result = self._parse_plan_response(raw, instruction)
        if plan_result:
            return plan_result
        return {"success": False, "error": "La IA no devolvió un JSON de plan válido"}

    def _parse_plan_response(self, raw: str, instruction: str) -> dict | None:
        """Extrae y normaliza JSON del texto de respuesta del LLM."""
        candidates = [raw.strip()]
        fenced = re.findall(r"```(?:json)?\s*(.*?)```", raw, flags=re.DOTALL | re.IGNORECASE)
        candidates.extend(chunk.strip() for chunk in fenced if chunk.strip())
        for candidate in candidates:
            try:
                plan = json.loads(candidate)
                normalized = self._normalize_plan(plan, instruction)
                return {"success": True, "plan": normalized}
            except Exception:
                continue
        return None

    def _normalize_plan(self, plan: dict, instruction: str) -> dict:
        if not isinstance(plan, dict):
            raise ValueError("El plan debe ser un JSON object")
        steps = list(plan.get("steps") or [])
        plan["instruction"] = plan.get("instruction") or instruction
        plan["intent"] = plan.get("intent") or instruction[:120]
        plan["parsed_at"] = datetime.now().isoformat()
        plan["requires_browser"] = bool(plan.get("requires_browser"))
        plan["requires_smart_browser"] = bool(plan.get("requires_smart_browser"))
        plan["requires_desktop"] = bool(plan.get("requires_desktop"))
        plan["requires_vision"] = bool(plan.get("requires_vision"))
        plan["risk_level"] = str(plan.get("risk_level") or "low").lower()
        plan["requires_confirmation"] = bool(plan.get("requires_confirmation"))

        if plan["requires_smart_browser"]:
            steps = [
                {
                    "step_number": 1,
                    "description": "Ejecutar la tarea completa con browser inteligente",
                    "action": "smart_browser_task",
                    "params": {"task": instruction},
                    "wait_after_ms": 1000,
                    "verify_with_vision": False,
                    "vision_check": "",
                    "on_failure": "abort",
                }
            ]

        normalized_steps = []
        for idx, step in enumerate(steps, start=1):
            if not isinstance(step, dict):
                continue
            action = str(step.get("action") or "").strip()
            action = ACTION_ALIASES.get(action, action)
            if action == "screenshot" and plan.get("requires_desktop") and not plan.get("requires_browser"):
                action = "desktop_screenshot"
            if action not in SUPPORTED_ACTIONS:
                raise ValueError(f"Accion no soportada: {action}")
            normalized_steps.append(
                {
                    "step_number": idx,
                    "description": step.get("description") or f"Paso {idx}",
                    "action": action,
                    "params": step.get("params") or {},
                    "wait_after_ms": int(step.get("wait_after_ms", 500) or 0),
                    "verify_with_vision": bool(step.get("verify_with_vision", False)),
                    "vision_check": step.get("vision_check") or "",
                    "on_failure": str(step.get("on_failure") or "retry").lower(),
                }
            )

        if not normalized_steps:
            raise ValueError("El plan no contiene pasos ejecutables")

        plan["steps"] = normalized_steps
        plan["success_criteria"] = plan.get("success_criteria") or "La tarea logra el objetivo sin errores críticos."
        plan["output_type"] = plan.get("output_type") or "summary"
        return plan

    def explain_plan(self, plan: dict) -> str:
        steps = plan.get("steps", [])
        lines = [
            f"📋 PLAN: {plan.get('intent','')}",
            f"🔢 Pasos: {len(steps)}",
            f"🌐 Browser simple: {'Sí' if plan.get('requires_browser') else 'No'}",
            f"🤖 Browser con IA: {'Sí' if plan.get('requires_smart_browser') else 'No'}",
            f"🖥️  Desktop: {'Sí' if plan.get('requires_desktop') else 'No'}",
            f"👁️  Visión: {'Sí' if plan.get('requires_vision') else 'No'}",
            f"⚠️  Riesgo: {plan.get('risk_level','low').upper()}",
            "",
            "PASOS:",
        ]
        for s in steps:
            lines.append(f"  {s['step_number']}. {s['description']}")
            lines.append(f"     → {s['action']}({s.get('params',{})})")
        lines.append(f"\n✅ Éxito cuando: {plan.get('success_criteria','')}")
        return "\n".join(lines)


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    parser = NLParser()
    test = parser.parse_instruction("Abre Google y busca el precio del bitcoin hoy")
    if test["success"]:
        print(parser.explain_plan(test["plan"]))
        print("OK NLParser")
    else:
        print("WARN NLParser error:", test["error"])
