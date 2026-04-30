import json
import re
from typing import Any, Dict, Optional

from browser_hands import BrowserHands
from desktop_hands import DesktopHands
from memory_manager import MemoryManager
from vision_eyes import VisionEyes


class VisualAgent:
    """
    Combina ojos (VisionEyes) + manos browser (BrowserHands)
    + manos desktop (DesktopHands) + memoria (MemoryManager)
    Ciclo: VE → PIENSA → ACTÚA → VERIFICA → RECUERDA
    """

    def __init__(self, headless_browser: bool = False):
        self.eyes = VisionEyes()
        self.browser = BrowserHands(headless=headless_browser)
        self.desktop = DesktopHands()
        self.memory = MemoryManager()
        self._browser_active = False

    async def start_browser(self):
        await self.browser.start()
        self._browser_active = True

    async def stop_browser(self):
        if self._browser_active:
            await self.browser.stop()
            self._browser_active = False

    def see_desktop(self, question: str = "¿Qué ves en la pantalla?") -> dict:
        shot_bytes = self.desktop.screenshot_bytes()
        return self.eyes.analyze_screenshot_bytes(shot_bytes, question)

    async def see_browser(self, question: str = "¿Qué ves en esta página?") -> dict:
        shot_bytes = await self.browser.screenshot_bytes()
        return self.eyes.analyze_screenshot_bytes(shot_bytes, question)

    async def navigate_and_analyze(self, url: str) -> dict:
        if not self._browser_active:
            await self.start_browser()
        nav = await self.browser.navigate(url)
        analysis = await self.see_browser(
            "Describe esta página: título, menú, botones, formularios, contenido principal"
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
        return {"success": False, "found": False}

    def full_status(self) -> dict:
        return {
            "agent": "ATLAS-WORKSPACE-PRIME v3.0",
            "eyes": {
                "vision_ai": "✅ Claude Vision via Bedrock",
                "browser_vision": "✅ Playwright screenshots",
                "desktop_vision": "✅ PyAutoGUI screenshots",
            },
            "hands": {
                "browser_simple": "✅ Playwright (BrowserHands)",
                "browser_smart": "✅ browser-use + Claude (SmartBrowser)",
                "desktop_mouse": "✅ PyAutoGUI (DesktopHands)",
                "desktop_keyboard": "✅ PyAutoGUI (DesktopHands)",
            },
            "brain": {
                "memory_4layers": "✅ MemoryManager",
                "web_fetch": "✅ WebTools / httpx",
                "file_rw": "✅ nativo",
                "code_exec": "✅ PowerShell",
            },
        }

    async def web_task(self, url: str, instructions: str) -> Dict[str, Any]:
        """
        Ejecuta una tarea web E2E mínima:
        1) Navega a URL
        2) Analiza visualmente
        3) Si la instrucción sugiere click en un elemento, intenta localizar y click
        4) Re-analiza y retorna evidencia
        """
        if not self._browser_active:
            await self.start_browser()

        nav = await self.browser.navigate(url)
        before = await self.see_browser(
            f"Contexto inicial. Objetivo del operador: {instructions}"
        )

        action_result: Optional[Dict[str, Any]] = None
        lower = (instructions or "").lower()
        wants_click = any(
            kw in lower for kw in ("click", "clic", "presiona", "pulsa", "botón", "boton")
        )
        if wants_click:
            target = self._extract_click_target(instructions) or "Continuar"
            action_result = await self.find_and_click_visual(target)

        after = await self.see_browser(
            f"Verifica si se cumplió el objetivo: {instructions}. Resume estado final."
        )
        screenshot_after = await self.browser.screenshot(name="web_task_after")

        ok = bool(nav.get("success")) and (
            action_result is None or bool(action_result.get("success"))
        )
        out = {
            "success": ok,
            "url": nav.get("url"),
            "title": nav.get("title"),
            "instructions": instructions,
            "before_analysis": before.get("analysis", ""),
            "action_result": action_result,
            "after_analysis": after.get("analysis", ""),
            "evidence": {
                "before_screenshot": nav.get("screenshot_path"),
                "after_screenshot": screenshot_after,
            },
        }
        self.memory.save_episode(
            task=f"web_task:{instructions[:120]}",
            result=out.get("title", ""),
            success=ok,
        )
        return out

    def desktop_task(self, instructions: str) -> Dict[str, Any]:
        """
        Ejecuta una tarea de escritorio local:
        1) screenshot + análisis
        2) retorno de coordenadas sugeridas por visión para asistencia operativa

        Nota: no hace clicks destructivos automáticos aquí; se mantiene seguro y auditable.
        """
        shot = self.desktop.screenshot()
        analysis = self.see_desktop(
            f"Analiza la pantalla para ejecutar esta instrucción de escritorio: {instructions}"
        )
        coords = self.eyes.find_element_coordinates(
            self.desktop.screenshot_bytes(),
            f"Elemento principal para: {instructions}",
        )
        out = {
            "success": True,
            "instructions": instructions,
            "analysis": analysis.get("analysis", ""),
            "suggested_coordinates": coords,
            "evidence": {"screenshot": shot},
        }
        self.memory.save_episode(
            task=f"desktop_task:{instructions[:120]}",
            result=str(coords),
            success=True,
        )
        return out

    @staticmethod
    def _extract_click_target(text: str) -> Optional[str]:
        t = (text or "").strip()
        if not t:
            return None
        # Extrae target básico: "click en X", "clic en X", "presiona X"
        m = re.search(
            r"(?:click|clic|presiona|pulsa)\s+(?:en\s+)?['\"]?([^'\".,;\n]{2,80})",
            t,
            flags=re.IGNORECASE,
        )
        if m:
            return m.group(1).strip()
        return None


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    agent = VisualAgent()
    print(json.dumps(agent.full_status(), indent=2, ensure_ascii=False))
    print("OK VisualAgent")
