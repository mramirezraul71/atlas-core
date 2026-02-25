import asyncio
import json
import sys
from datetime import datetime
from pathlib import Path

from desktop_hands import DesktopHands
from memory_manager import MemoryManager
from nl_parser import NLParser
from plan_executor import PlanExecutor
from vision_eyes import VisionEyes

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from modules.humanoid.ai.router import route_and_run


class AtlasPrime:
    """
    ATLAS-WORKSPACE-PRIME — Interfaz principal de lenguaje natural

    Uso:
        atlas = AtlasPrime()
        await atlas.run("Entra a Gmail y dime mis correos de hoy")
        await atlas.run("Abre el explorador y organiza mis archivos de escritorio")
        await atlas.run("Busca en Google el precio del dólar y guárdalo en un archivo")
    """

    def __init__(self, headless_browser: bool = False, auto_confirm: bool = False):
        self.parser = NLParser()
        self.memory = MemoryManager()
        self.eyes = VisionEyes()
        self.desktop = DesktopHands()
        self.headless = headless_browser
        self.auto_confirm = auto_confirm
        print("🤖 ATLAS-WORKSPACE-PRIME v3.0 iniciado")

    async def run(
        self,
        instruction: str,
        show_plan: bool = True,
        take_context_screenshot: bool = True,
    ) -> dict:
        """
        Ejecuta cualquier instrucción en lenguaje natural.
        Este es el método principal — úsalo para todo.
        """
        print(f"\n{'='*60}")
        print(f"📥 INSTRUCCIÓN: {instruction}")
        print(f"{'='*60}")

        # 1. Capturar contexto visual actual
        screen_context = ""
        if take_context_screenshot:
            print("👁️  Analizando pantalla actual...")
            vision = self.eyes.analyze_screenshot_bytes(
                self.desktop.screenshot_bytes(),
                "Describe brevemente qué hay en pantalla ahora mismo: aplicaciones abiertas, ventanas visibles",
            )
            screen_context = vision.get("analysis", "")
            print(f"   Contexto: {screen_context[:100]}...")

        # 2. Parsear instrucción a plan
        print("🧠 Generando plan de ejecución...")
        parsed = self.parser.parse_instruction(instruction, screen_context)

        if not parsed["success"]:
            return {
                "success": False,
                "error": f"No pude generar plan: {parsed['error']}",
            }

        plan = parsed["plan"]

        # 3. Mostrar plan al usuario
        if show_plan:
            print("\n" + self.parser.explain_plan(plan))

        # 4. Confirmar si es necesario
        if plan.get("requires_confirmation") and not self.auto_confirm:
            print("\n❓ Esta tarea requiere confirmación. ¿Ejecutar? (s/n): ", end="")
            if input().strip().lower() != "s":
                return {"success": False, "reason": "Cancelado por usuario"}

        # 5. Ejecutar plan
        print("\n🚀 Iniciando ejecución...")
        executor = PlanExecutor(headless_browser=self.headless)
        result = await executor.execute_plan(
            plan, confirm_before_high_risk=not self.auto_confirm
        )

        # 6. Generar resumen final con IA
        if result.get("final_output"):
            summary = await self._summarize_result(instruction, result)
            result["summary"] = summary
            print(f"\n📊 RESUMEN:\n{summary}")

        # 7. Guardar en memoria
        self.memory.save_episode(
            task=instruction,
            result=result.get("summary", str(result))[:200],
            success=result.get("success", False),
        )

        return result

    async def _summarize_result(self, instruction: str, result: dict) -> str:
        """Resume el resultado final en lenguaje natural"""
        content = result.get("final_output", {})
        prompt = f"""
Tarea ejecutada: {instruction}
Pasos completados: {result.get('steps_executed', 0)}
Contenido obtenido: {str(content.get('page_content', ''))[:1000]}
Análisis visual: {str(content.get('last_analysis', ''))[:500]}

Resume en 3-5 líneas en español qué se logró y cuáles son los datos más importantes obtenidos.
"""

        try:
            out, _decision, _meta = route_and_run(
                prompt=prompt,
                intent_hint="chat",
                modality="text",
                prefer_free=False,
            )
            return (
                out or ""
            ).strip() or f"Tarea completada en {result.get('steps_executed', 0)} pasos."
        except Exception:
            return f"Tarea completada en {result.get('steps_executed', 0)} pasos."

    def see_now(self, question: str = "¿Qué ves en pantalla ahora mismo?") -> str:
        """Mira la pantalla ahora mismo y describe lo que ve"""
        vision = self.eyes.analyze_screenshot_bytes(
            self.desktop.screenshot_bytes(), question
        )
        return vision.get("analysis", "No pude analizar la pantalla")

    def recent_history(self, n: int = 5) -> list:
        """Retorna las últimas N tareas ejecutadas"""
        return self.memory.get_recent_episodes(n)


# ══════════════════════════════════════════════════════════════
# MODO INTERACTIVO — Ejecuta atlas_prime.py y escribe tareas
# ══════════════════════════════════════════════════════════════
async def interactive_mode():
    atlas = AtlasPrime(headless_browser=False, auto_confirm=False)

    print("\n" + "=" * 60)
    print("🤖 ATLAS-WORKSPACE-PRIME — MODO INTERACTIVO")
    print("=" * 60)
    print("Escribe cualquier instrucción en español.")
    print("Comandos especiales:")
    print("  'ver'      → ve la pantalla ahora mismo")
    print("  'historial'→ muestra tareas recientes")
    print("  'salir'    → termina el programa")
    print("=" * 60 + "\n")

    while True:
        try:
            instruction = input("📥 Tu instrucción: ").strip()
            if not instruction:
                continue
            elif instruction.lower() == "salir":
                print("👋 Hasta luego")
                break
            elif instruction.lower() == "ver":
                print(atlas.see_now())
            elif instruction.lower() == "historial":
                history = atlas.recent_history()
                for ep in history:
                    status = "✅" if ep.get("success") else "❌"
                    print(f"{status} [{ep['timestamp'][:16]}] {ep['task'][:60]}")
            else:
                result = await atlas.run(instruction)
                if not result.get("success"):
                    print(f"❌ Error: {result.get('error') or result.get('reason')}")
        except KeyboardInterrupt:
            print("\n👋 Interrumpido")
            break


if __name__ == "__main__":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    asyncio.run(interactive_mode())
