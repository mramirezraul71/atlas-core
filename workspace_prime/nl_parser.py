import json
from datetime import datetime

import load_credentials  # noqa: F401 — carga CREDENTIALS_FILE y .env
from llm_router import text_completion


class NLParser:
    """Convierte instrucciones en lenguaje natural a planes ejecutables. Usa todas las IAs disponibles (Bedrock→Anthropic→OpenAI→Gemini→Groq→DeepSeek)."""

    def __init__(self):
        pass

    def parse_instruction(self, instruction: str, screen_context: str = "") -> dict:
        system = """
Eres el planificador de ATLAS-WORKSPACE-PRIME.
Conviertes instrucciones humanas en planes de ejecución JSON.

Capacidades disponibles:
BROWSER_SIMPLE: navigate(url), click(selector), type_text(selector,text),
                press_key(key), scroll(direction), get_page_content(),
                execute_js(script), go_back(), new_tab(url)
BROWSER_SMART:  smart_browser_task(task) — para sitios complejos con IA
DESKTOP:        desktop_click(x,y), desktop_type(text), hotkey(keys[]),
                open_app(path), screenshot()
VISION:         see_browser(question), see_desktop(question),
                find_and_click_visual(element_description)
FILES:          read_file(path), write_file(path,content),
                execute_command(cmd)
MEMORY:         memory_read(), memory_write(key,value)

Responde SOLO con JSON válido sin texto adicional:
{
  "instruction": "instrucción original",
  "intent": "qué hace en una frase",
  "requires_browser": true,
  "requires_smart_browser": false,
  "requires_desktop": false,
  "requires_vision": false,
  "steps": [
    {
      "step_number": 1,
      "description": "descripción en español",
      "action": "nombre_metodo",
      "params": {},
      "wait_after_ms": 1000,
      "verify_with_vision": false,
      "vision_check": "",
      "on_failure": "retry"
    }
  ],
  "success_criteria": "cómo sé que terminó bien",
  "output_type": "screenshot|text|file|summary",
  "risk_level": "low|medium|high",
  "requires_confirmation": false
}
"""
        user_msg = f"Instrucción: {instruction}"
        if screen_context:
            user_msg += f"\n\nContexto visual actual:\n{screen_context}"

        out = text_completion(user_msg, system=system, max_tokens=2048)
        if not out.get("success"):
            return {"success": False, "error": out.get("error", "Unknown error")}
        raw = (out.get("text") or "").strip()
        plan_result = self._parse_plan_response(raw)
        if plan_result:
            return plan_result
        return {"success": False, "error": "La IA no devolvió un JSON de plan válido"}

    def _parse_plan_response(self, raw: str) -> dict | None:
        """Extrae JSON del texto de respuesta del LLM. None si falla."""
        try:
            if raw.startswith("```"):
                raw = raw.split("```")[1]
                if raw.startswith("json"):
                    raw = raw[4:]
            plan = json.loads(raw.strip())
            plan["parsed_at"] = datetime.now().isoformat()
            return {"success": True, "plan": plan}
        except Exception:
            return None

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
