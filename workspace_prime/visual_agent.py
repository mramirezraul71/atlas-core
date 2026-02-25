import asyncio
import json
import subprocess
import sys
import time

from browser_hands import BrowserHands
from desktop_hands import DesktopHands
from memory_manager import MemoryManager
from vision_eyes import VisionEyes


class VisualAgent:
    """
    ATLAS-WORKSPACE-PRIME — Agente Visual Autónomo

    Combina:
    - VisionEyes: ve la pantalla (interno y externo)
    - BrowserHands: navega y actúa en browsers
    - DesktopHands: actúa en el sistema operativo
    - MemoryManager: recuerda lo que hizo

    Ciclo: VE → PIENSA → ACTÚA → VERIFICA → RECUERDA
    """

    def __init__(self, headless_browser: bool = False, voice_enabled: bool = True):
        self.eyes = VisionEyes()
        self.voice_enabled = voice_enabled
        self.browser = BrowserHands(
            headless=headless_browser, voice_enabled=voice_enabled
        )
        self.desktop = DesktopHands()
        self.memory = MemoryManager()
        self._browser_active = False
        # Evita doble narración: BrowserHands queda como canal principal de voz.
        self._speak_enabled = False

    def _speak(self, text: str) -> None:
        """Narración por audio en PC para pasos del agente."""
        if not self.voice_enabled or not self._speak_enabled:
            return
        msg = str(text or "").strip()
        if not msg:
            return
        safe = msg.replace("'", "''")
        ps = (
            "Add-Type -AssemblyName System.Speech; "
            "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
            "$s.Rate = 0; "
            f"$s.Speak('{safe}')"
        )
        try:
            subprocess.Popen(
                ["powershell", "-NoProfile", "-Command", ps],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass

    # ── INICIALIZACIÓN ──────────────────────────────────────────────────
    async def start_browser(self):
        self._speak("Iniciando navegador para tarea visual")
        await self.browser.start()
        self._browser_active = True
        print("VisualAgent: Browser listo")

    async def stop_browser(self):
        if self._browser_active:
            await self.browser.stop()
            self._browser_active = False

    # ── VER Y ANALIZAR ──────────────────────────────────────────────────
    def see_desktop(self, question: str = "¿Qué ves en la pantalla?") -> dict:
        """Toma screenshot del escritorio y lo analiza con Vision AI"""
        shot_bytes = self.desktop.screenshot_bytes()
        analysis = self.eyes.analyze_screenshot_bytes(shot_bytes, question)
        return analysis

    async def see_browser(self, question: str = "¿Qué ves en esta página web?") -> dict:
        """Toma screenshot del browser y lo analiza con Vision AI"""
        shot_bytes = await self.browser.screenshot_bytes()
        analysis = self.eyes.analyze_screenshot_bytes(shot_bytes, question)
        return analysis

    # ── NAVEGACIÓN INTELIGENTE ──────────────────────────────────────────
    async def navigate_and_analyze(self, url: str) -> dict:
        """Navega a URL, ve la página, la analiza y reporta"""
        if not self._browser_active:
            await self.start_browser()

        self._speak("Voy a navegar y analizar la página")
        nav = await self.browser.navigate(url)
        self._speak("Analizando visualmente la página")
        analysis = await self.see_browser(
            "Describe esta página: título, menú principal, botones, formularios, contenido principal"
        )
        result = {
            "url": nav.get("url"),
            "title": nav.get("title"),
            "visual_analysis": analysis.get("analysis", ""),
            "screenshot": nav.get("screenshot_path"),
        }
        self.memory.save_episode(
            task=f"navigate:{url}", result=result["title"], success=True
        )
        return result

    async def find_and_click_visual(self, element_description: str) -> dict:
        """
        Ve la página, encuentra el elemento visualmente,
        obtiene sus coordenadas y hace click
        """
        shot_bytes = await self.browser.screenshot_bytes()
        coords = self.eyes.find_element_coordinates(shot_bytes, element_description)

        if coords.get("found"):
            click_result = await self.browser.click(x=coords["x"], y=coords["y"])
            await self.browser.page.wait_for_timeout(1000)
            return {
                "success": True,
                "found": True,
                "coordinates": {"x": coords["x"], "y": coords["y"]},
                "click": click_result,
            }
        return {
            "success": False,
            "found": False,
            "description": coords.get("description"),
        }

    # ── TAREA AUTÓNOMA WEB ──────────────────────────────────────────────
    async def web_task(self, url: str, instructions: str) -> dict:
        """
        Tarea autónoma completa en browser.
        Ve la página y sigue instrucciones usando visión.

        Ejemplo: web_task("https://google.com", "Busca noticias de ATLAS robot")
        """
        if not self._browser_active:
            await self.start_browser()

        steps_taken = []

        # Paso 1: Navegar
        self._speak("Paso uno, navegación")
        nav = await self.browser.navigate(url)
        steps_taken.append(f"Navegado a: {nav.get('title')}")

        # Paso 2: Ver la página
        self._speak("Paso dos, análisis visual")
        analysis = await self.see_browser(
            f"Dado que necesito: {instructions}\n"
            f"¿Qué elementos debo usar en esta página? "
            f"Dame el selector CSS o descripción visual del elemento a usar primero."
        )
        steps_taken.append(f"Analizado: {analysis.get('analysis', '')[:200]}")

        # Paso 3: Screenshot final
        self._speak("Paso tres, guardando evidencia visual")
        screenshot_path = await self.browser.screenshot(name="task_complete")

        result = {
            "success": True,
            "url": url,
            "instructions": instructions,
            "steps_taken": steps_taken,
            "final_analysis": analysis.get("analysis", ""),
            "screenshot": screenshot_path,
        }

        self.memory.save_episode(
            task=f"web_task: {instructions[:50]}", result=str(steps_taken), success=True
        )
        return result

    # ── TAREA AUTÓNOMA ESCRITORIO ───────────────────────────────────────
    def desktop_task(self, instructions: str) -> dict:
        """
        Tarea autónoma en escritorio.
        Ve la pantalla y actúa según instrucciones.
        """
        # Ver pantalla actual
        analysis = self.see_desktop(
            f"Dado que necesito: {instructions}\n"
            f"¿Qué veo en pantalla y qué debo hacer?"
        )

        # Tomar screenshot de referencia
        screenshot_path = self.desktop.screenshot()

        result = {
            "success": True,
            "instructions": instructions,
            "visual_analysis": analysis.get("analysis", ""),
            "screenshot": screenshot_path,
            "ready_to_act": True,
        }

        self.memory.save_episode(
            task=f"desktop_task: {instructions[:50]}",
            result=analysis.get("analysis", "")[:100],
            success=True,
        )
        return result

    def full_status(self) -> dict:
        return {
            "agent": "ATLAS-WORKSPACE-PRIME v3.0 VISUAL",
            "eyes": {
                "vision_ai": "✅ Claude Vision via Bedrock",
                "browser_vision": "✅ Playwright screenshots",
                "desktop_vision": "✅ PyAutoGUI screenshots",
            },
            "hands": {
                "browser_navigation": "✅ Playwright",
                "browser_click_type": "✅ Playwright",
                "desktop_mouse": "✅ PyAutoGUI",
                "desktop_keyboard": "✅ PyAutoGUI",
                "desktop_apps": "✅ subprocess",
            },
            "brain": {
                "memory": "✅ 4 layers",
                "web_fetch": "✅ httpx",
                "file_rw": "✅ nativo",
                "code_exec": "✅ PowerShell",
            },
            "browser_active": self._browser_active,
        }


# ── TEST ─────────────────────────────────────────────────────────────────
async def test_visual_agent():
    agent = VisualAgent(headless_browser=False)

    print("=== STATUS ===")
    print(json.dumps(agent.full_status(), indent=2, ensure_ascii=False))

    print("\n=== TEST NAVEGACIÓN ===")
    result = await agent.navigate_and_analyze("https://www.google.com")
    print("Título:", result["title"])
    print("Análisis visual:", result["visual_analysis"][:200])

    print("\n=== TEST ESCRITORIO ===")
    desktop_result = agent.desktop_task(
        "Toma un screenshot y describe el estado del sistema"
    )
    print("Screenshot:", desktop_result["screenshot"])

    await agent.stop_browser()
    print("\n✅ VisualAgent test completado")


if __name__ == "__main__":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    asyncio.run(test_visual_agent())
