import asyncio
import os

from dotenv import load_dotenv

load_dotenv()


class SmartBrowser:
    """
    browser-use + cualquier IA disponible (Anthropic → OpenAI → Gemini).
    Para tareas web complejas en lenguaje natural.
    """

    def __init__(self):
        self._available = self._check_available()

    def _check_available(self) -> bool:
        try:
            import browser_use

            return True
        except ImportError:
            return False

    async def run_task(self, task: str, model: str = None) -> dict:
        """
        Ejecuta cualquier tarea web en lenguaje natural.
        Usa la primera IA disponible: Anthropic → OpenAI → Gemini.
        """
        if not self._available:
            return {
                "success": False,
                "error": "browser-use no instalado. Ejecuta: pip install browser-use",
            }
        from llm_router import get_langchain_llm

        llm, engine = get_langchain_llm()
        if not llm:
            return {
                "success": False,
                "error": "Ninguna IA configurada (ANTHROPIC_API_KEY, OPENAI_API_KEY o GEMINI_API_KEY)",
            }
        try:
            from browser_use import Agent

            agent = Agent(task=task, llm=llm)
            result = await agent.run()
            return {
                "success": True,
                "task": task,
                "result": str(result.final_result()) if result else "Completado",
                "engine": f"browser-use + {engine}",
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def run_task_openai(self, task: str, model: str = "gpt-4o") -> dict:
        """Alias: mismo que run_task (usa router multi-IA)."""
        return await self.run_task(task, model)


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    async def test():
        sb = SmartBrowser()
        if sb._available:
            print("OK SmartBrowser - browser-use disponible")
        else:
            print("WARN SmartBrowser: browser-use no instalado")

    asyncio.run(test())
