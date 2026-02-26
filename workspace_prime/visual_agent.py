import asyncio
import json

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
