import asyncio
import io
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import json

import load_credentials  # noqa: F401 — carga credenciales antes de boto3
from desktop_hands import DesktopHands
from memory_manager import MemoryManager
from nl_parser import NLParser
from plan_executor import PlanExecutor
from smart_browser import SmartBrowser
from vision_eyes import VisionEyes


class AtlasPrime:
    """
    ATLAS-WORKSPACE-PRIME v3.0 — Interfaz principal de lenguaje natural.

    Elige automáticamente la herramienta correcta:
    - Tarea simple web    → Playwright (BrowserHands)
    - Tarea compleja web  → browser-use + Claude (SmartBrowser)
    - Tarea de escritorio → PyAutoGUI (DesktopHands)
    - Análisis visual     → Claude Vision vía Bedrock (VisionEyes)
    - Todo persistido     → MemoryManager (4 capas)

    Uso:
        atlas = AtlasPrime()
        await atlas.run("Entra a Gmail y dime mis correos de hoy")
    """

    def __init__(self, headless_browser: bool = False, auto_confirm: bool = False):
        self.parser = NLParser()
        self.memory = MemoryManager()
        self.eyes = VisionEyes()
        self.desktop = DesktopHands()
        self.smart = SmartBrowser()
        self.headless = headless_browser
        self.auto_confirm = auto_confirm
        print("🤖 ATLAS-WORKSPACE-PRIME v3.0 iniciado")
        print(
            f"   Smart Browser: {'✅ disponible' if self.smart._available else '⚠️ instalar browser-use'}"
        )

    async def run(
        self,
        instruction: str,
        show_plan: bool = True,
        take_context: bool = True,
        force_smart: bool = False,
    ) -> dict:
        """
        Ejecuta cualquier instrucción en lenguaje natural.

        force_smart=True → usa browser-use directamente sin planificar
        """
        print(f"\n{'='*60}")
        print(f"📥 INSTRUCCIÓN: {instruction}")
        print(f"{'='*60}")

        # Modo fast: browser-use directo sin pasos intermedios
        if force_smart and self.smart._available:
            print("🤖 Modo SmartBrowser directo (browser-use + Claude)...")
            result = await self.smart.run_task(instruction)
            self.memory.save_episode(
                task=instruction,
                result=result.get("result", "")[:200],
                success=result.get("success", False),
            )
            if result.get("success"):
                print(f"\n✅ RESULTADO:\n{result.get('result','')}")
            return result

        # Modo normal: NLParser → PlanExecutor
        screen_context = ""
        if take_context:
            print("👁️  Analizando pantalla actual...")
            vision = self.eyes.analyze_screenshot_bytes(
                self.desktop.screenshot_bytes(),
                "Describe brevemente qué hay en pantalla: apps abiertas, ventanas",
            )
            screen_context = vision.get("analysis", "")
            print(f"   Contexto: {screen_context[:100]}...")

        print("🧠 Generando plan...")
        parsed = self.parser.parse_instruction(instruction, screen_context)

        if not parsed["success"]:
            # Fallback a SmartBrowser si falla el parser
            if self.smart._available:
                print("⚠️  Parser falló, usando SmartBrowser como fallback...")
                return await self.smart.run_task(instruction)
            return {"success": False, "error": parsed["error"]}

        plan = parsed["plan"]

        if show_plan:
            print("\n" + self.parser.explain_plan(plan))

        if plan.get("requires_confirmation") and not self.auto_confirm:
            if sys.stdin.isatty():
                print("\n❓ ¿Ejecutar? (s/n): ", end="")
                if input().strip().lower() != "s":
                    return {"success": False, "reason": "Cancelado"}
            else:
                pass  # Sin TTY: continuar sin preguntar

        executor = PlanExecutor(headless_browser=self.headless)
        result = await executor.execute_plan(
            plan, confirm_high_risk=not self.auto_confirm
        )

        if result.get("final_output"):
            summary = await self._summarize(instruction, result)
            result["summary"] = summary
            print(f"\n📊 RESUMEN:\n{summary}")

        self.memory.save_episode(
            task=instruction,
            result=result.get("summary", "")[:200],
            success=result.get("success", False),
        )
        return result

    async def _summarize(self, instruction: str, result: dict) -> str:
        from llm_router import text_completion

        content = result.get("final_output", {})
        user_msg = f"""
Tarea: {instruction}
Pasos completados: {result.get('steps_executed', 0)}
Contenido: {str(content.get('page_content',''))[:1000]}
Análisis: {str(content.get('last_analysis',''))[:500]}
Resultado smart: {str(content.get('smart_result',''))[:500]}

Resume en 3-5 líneas qué se logró y los datos más importantes.
"""
        out = text_completion(
            user_msg,
            system="Eres un asistente que resume tareas completadas.",
            max_tokens=500,
        )
        if out.get("success"):
            return out.get("text", "").strip()
        return f"Completado en {result.get('steps_executed', 0)} pasos."

    def see_now(self, question: str = "¿Qué hay en pantalla ahora mismo?") -> str:
        vision = self.eyes.analyze_screenshot_bytes(
            self.desktop.screenshot_bytes(), question
        )
        return vision.get("analysis", "No pude analizar la pantalla")

    def history(self, n: int = 5) -> list:
        return self.memory.get_recent_episodes(n)

    def status(self) -> dict:
        from visual_agent import VisualAgent

        va = VisualAgent()
        return va.full_status()


# ══════════════════════════════════════════════════════════════════
# MODO INTERACTIVO
# ══════════════════════════════════════════════════════════════════
async def run_single_cmd(cmd: str) -> None:
    """Ejecuta un solo comando sin TTY (para scripts/agente)."""
    atlas = AtlasPrime(headless_browser=True, auto_confirm=True)
    c = cmd.strip()
    if not c:
        return
    if c.lower() == "ver":
        print(atlas.see_now())
    elif c.lower() == "historial":
        for ep in atlas.history():
            icon = "OK" if ep.get("success") else "FAIL"
            print(f"{icon} [{ep['timestamp'][:16]}] {ep['task'][:60]}")
    elif c.lower() == "status":
        print(json.dumps(atlas.status(), indent=2, ensure_ascii=False))
    elif c.lower().startswith("smart "):
        task = c[6:].strip()
        result = await atlas.run(task, force_smart=True)
        if result.get("success"):
            print(result.get("result", ""))
        else:
            print("ERROR:", result.get("error") or result.get("reason"))
    else:
        result = await atlas.run(c)
        if result.get("success") and result.get("summary"):
            print(result.get("summary", ""))
        elif not result.get("success"):
            print("ERROR:", result.get("error") or result.get("reason"))


async def interactive_mode():
    if not sys.stdin.isatty():
        print("Modo interactivo requiere terminal (stdin no es TTY).")
        print("Opciones: -h/--help (ayuda), --version (versión).")
        return
    atlas = AtlasPrime(headless_browser=False, auto_confirm=False)

    print("\n" + "=" * 60)
    print("🤖 ATLAS-WORKSPACE-PRIME v3.0 — MODO INTERACTIVO")
    print("=" * 60)
    print("Escribe cualquier instrucción en español.")
    print("Comandos:")
    print("  'ver'       → analiza la pantalla ahora mismo")
    print("  'smart X'   → ejecuta X con browser-use directo")
    print("  'historial' → últimas tareas")
    print("  'status'    → estado del sistema")
    print("  'salir'     → termina")
    print("=" * 60 + "\n")

    while True:
        try:
            cmd = input("📥 Tu instrucción: ").strip()
            if not cmd:
                continue
            elif cmd.lower() == "salir":
                print("👋 Hasta luego")
                break
            elif cmd.lower() == "ver":
                print(atlas.see_now())
            elif cmd.lower() == "historial":
                for ep in atlas.history():
                    icon = "✅" if ep.get("success") else "❌"
                    print(f"{icon} [{ep['timestamp'][:16]}] {ep['task'][:60]}")
            elif cmd.lower() == "status":
                print(json.dumps(atlas.status(), indent=2, ensure_ascii=False))
            elif cmd.lower().startswith("smart "):
                task = cmd[6:].strip()
                result = await atlas.run(task, force_smart=True)
                if not result.get("success"):
                    print(f"❌ {result.get('error') or result.get('reason')}")
            else:
                result = await atlas.run(cmd)
                if not result.get("success"):
                    print(f"❌ {result.get('error') or result.get('reason')}")
        except KeyboardInterrupt:
            print("\n👋 Interrumpido")
            break
        except EOFError:
            print("\nSin entrada (stdin cerrado). Saliendo.")
            break


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(
        description="ATLAS-WORKSPACE-PRIME v3.0 — Modo interactivo",
        epilog="Sin TTY: usa --cmd COMANDO para ejecutar una instrucción (ej: status, ver, historial).",
    )
    p.add_argument("--version", action="version", version="ATLAS-WORKSPACE-PRIME 3.0")
    p.add_argument(
        "--cmd",
        type=str,
        metavar="COMANDO",
        help="Ejecutar un solo comando sin terminal (status, ver, historial, o instrucción)",
    )
    args = p.parse_args()
    if args.cmd is not None:
        asyncio.run(run_single_cmd(args.cmd))
    else:
        asyncio.run(interactive_mode())
