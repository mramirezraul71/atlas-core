import asyncio
import json
import subprocess
from datetime import datetime
from pathlib import Path

from playwright.async_api import Browser, Page, async_playwright

SCREENSHOTS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\screenshots")
SCREENSHOTS_DIR.mkdir(parents=True, exist_ok=True)


class BrowserHands:
    """
    ATLAS-WORKSPACE-PRIME — Manos externas (browser)
    Navega, hace click, llena formularios, toma screenshots
    Ve el browser como un humano y actúa en él
    """

    def __init__(self, headless: bool = False, voice_enabled: bool = True):
        self.headless = headless  # False = puedes VER el browser
        self.voice_enabled = voice_enabled
        self.browser: Browser = None
        self.page: Page = None
        self._playwright = None
        self._voice_phrases = {
            "browser_opening": "Abriendo navegador.",
            "browser_ready": "Navegador listo.",
            "navigating": "Navegando a la página solicitada.",
            "page_loaded": "Página cargada.",
            "click_element": "Haciendo clic en el elemento.",
            "click_coordinates": "Haciendo clic en la posición indicada.",
            "typing": "Escribiendo en el campo.",
            "configuring_option": "Configurando una opción de la página.",
            "new_tab": "Abriendo una nueva pestaña.",
        }

    def _speak(self, text: str) -> None:
        """Voz local en Windows para narrar acciones."""
        if not self.voice_enabled:
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

    def _announce(self, key: str) -> None:
        self._speak(self._voice_phrases.get(key, "Ejecutando acción."))

    async def start(self):
        """Inicia el browser"""
        self._announce("browser_opening")
        self._playwright = await async_playwright().start()
        self.browser = await self._playwright.chromium.launch(
            headless=self.headless, args=["--start-maximized"]
        )
        context = await self.browser.new_context(
            viewport={"width": 1280, "height": 800},
            user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36",
        )
        self.page = await context.new_page()
        print("BrowserHands: Browser iniciado")
        self._announce("browser_ready")
        return self

    async def stop(self):
        if self.browser:
            await self.browser.close()
        if self._playwright:
            await self._playwright.stop()

    async def navigate(self, url: str) -> dict:
        """Navega a una URL y toma screenshot"""
        self._announce("navigating")
        await self.page.goto(url, wait_until="networkidle", timeout=30000)
        await self.page.wait_for_timeout(1000)
        screenshot = await self.screenshot()
        self._announce("page_loaded")
        return {
            "success": True,
            "url": self.page.url,
            "title": await self.page.title(),
            "screenshot_path": screenshot,
        }

    async def screenshot(self, name: str = None) -> str:
        """Toma screenshot y lo guarda"""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{name or 'screen'}_{ts}.png"
        path = str(SCREENSHOTS_DIR / filename)
        await self.page.screenshot(path=path, full_page=False)
        return path

    async def screenshot_bytes(self) -> bytes:
        """Screenshot como bytes para enviar a VisionEyes"""
        return await self.page.screenshot(type="png")

    async def click(self, selector: str = None, x: int = None, y: int = None) -> dict:
        """Click por selector CSS o por coordenadas x,y"""
        try:
            if selector:
                self._announce("click_element")
                await self.page.click(selector, timeout=5000)
                return {"success": True, "action": f"clicked selector: {selector}"}
            elif x and y:
                self._announce("click_coordinates")
                await self.page.mouse.click(x, y)
                return {"success": True, "action": f"clicked coordinates: ({x},{y})"}
            return {"success": False, "error": "Necesitas selector o coordenadas"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def type_text(
        self, selector: str, text: str, clear_first: bool = True
    ) -> dict:
        """Escribe texto en un campo"""
        try:
            self._announce("typing")
            if clear_first:
                await self.page.fill(selector, "")
            await self.page.type(selector, text, delay=50)
            return {"success": True, "action": f"typed in {selector}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def press_key(self, key: str) -> dict:
        """Presiona una tecla: Enter, Tab, Escape, ArrowDown, etc"""
        await self.page.keyboard.press(key)
        return {"success": True, "action": f"pressed {key}"}

    async def scroll(self, direction: str = "down", amount: int = 300) -> dict:
        """Scroll en la página"""
        if direction == "down":
            await self.page.mouse.wheel(0, amount)
        elif direction == "up":
            await self.page.mouse.wheel(0, -amount)
        return {"success": True, "action": f"scrolled {direction} {amount}px"}

    async def get_text(self, selector: str) -> dict:
        """Obtiene el texto de un elemento"""
        try:
            text = await self.page.inner_text(selector)
            return {"success": True, "text": text}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def get_page_content(self) -> dict:
        """Obtiene todo el texto visible de la página"""
        content = await self.page.inner_text("body")
        return {
            "success": True,
            "url": self.page.url,
            "title": await self.page.title(),
            "content": content[:5000],
        }

    async def wait_for_element(self, selector: str, timeout: int = 10000) -> dict:
        """Espera que aparezca un elemento"""
        try:
            await self.page.wait_for_selector(selector, timeout=timeout)
            return {"success": True, "found": True}
        except:
            return {"success": False, "found": False}

    async def hover(self, selector: str) -> dict:
        """Hover sobre elemento"""
        try:
            await self.page.hover(selector)
            return {"success": True, "action": f"hovered {selector}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def select_option(self, selector: str, value: str) -> dict:
        """Selecciona opción en dropdown"""
        try:
            self._announce("configuring_option")
            await self.page.select_option(selector, value)
            return {"success": True, "action": f"selected {value}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def go_back(self) -> dict:
        await self.page.go_back()
        return {"success": True, "url": self.page.url}

    async def go_forward(self) -> dict:
        await self.page.go_forward()
        return {"success": True, "url": self.page.url}

    async def new_tab(self, url: str = None) -> dict:
        """Abre nueva pestaña"""
        self._announce("new_tab")
        context = self.page.context
        new_page = await context.new_page()
        self.page = new_page
        if url:
            await self.navigate(url)
        return {"success": True, "action": "new tab opened"}

    async def execute_js(self, script: str) -> dict:
        """Ejecuta JavaScript en la página"""
        try:
            result = await self.page.evaluate(script)
            return {"success": True, "result": str(result)}
        except Exception as e:
            return {"success": False, "error": str(e)}


# ── FUNCIÓN SINCRÓNICA para llamar desde el agente ──────────────────────
def run_browser_task(task_fn):
    """Helper para correr tareas async del browser desde código sync"""
    return asyncio.run(task_fn)


if __name__ == "__main__":

    async def test():
        b = BrowserHands(headless=False)
        await b.start()
        result = await b.navigate("https://www.google.com")
        print("Navegación OK:", result["title"])
        await b.stop()

    asyncio.run(test())
