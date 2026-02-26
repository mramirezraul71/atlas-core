import asyncio
from datetime import datetime
from pathlib import Path

from playwright.async_api import Browser, Page, async_playwright

SCREENSHOTS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\screenshots")
SCREENSHOTS_DIR.mkdir(parents=True, exist_ok=True)


class BrowserHands:
    """Playwright directo — para tareas web simples y rápidas"""

    def __init__(self, headless: bool = False):
        self.headless = headless
        self.browser: Browser = None
        self.page: Page = None
        self._playwright = None

    async def start(self):
        self._playwright = await async_playwright().start()
        self.browser = await self._playwright.chromium.launch(
            headless=self.headless, args=["--start-maximized"]
        )
        context = await self.browser.new_context(
            viewport={"width": 1280, "height": 800},
            user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64)",
        )
        self.page = await context.new_page()
        return self

    async def stop(self):
        if self.browser:
            await self.browser.close()
        if self._playwright:
            await self._playwright.stop()

    async def navigate(self, url: str) -> dict:
        await self.page.goto(url, wait_until="networkidle", timeout=30000)
        await self.page.wait_for_timeout(1000)
        screenshot = await self.screenshot()
        return {
            "success": True,
            "url": self.page.url,
            "title": await self.page.title(),
            "screenshot_path": screenshot,
        }

    async def screenshot(self, name: str = None) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = str(SCREENSHOTS_DIR / f"{name or 'screen'}_{ts}.png")
        await self.page.screenshot(path=path, full_page=False)
        return path

    async def screenshot_bytes(self) -> bytes:
        return await self.page.screenshot(type="png")

    async def click(self, selector: str = None, x: int = None, y: int = None) -> dict:
        try:
            if selector:
                await self.page.click(selector, timeout=5000)
                return {"success": True, "action": f"clicked: {selector}"}
            elif x and y:
                await self.page.mouse.click(x, y)
                return {"success": True, "action": f"clicked: ({x},{y})"}
            return {"success": False, "error": "Necesitas selector o coordenadas"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def type_text(
        self, selector: str, text: str, clear_first: bool = True
    ) -> dict:
        try:
            if clear_first:
                await self.page.fill(selector, "")
            await self.page.type(selector, text, delay=50)
            return {"success": True, "action": f"typed in {selector}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def press_key(self, key: str) -> dict:
        await self.page.keyboard.press(key)
        return {"success": True, "action": f"pressed {key}"}

    async def scroll(self, direction: str = "down", amount: int = 300) -> dict:
        delta = amount if direction == "down" else -amount
        await self.page.mouse.wheel(0, delta)
        return {"success": True, "action": f"scrolled {direction}"}

    async def get_page_content(self) -> dict:
        content = await self.page.inner_text("body")
        return {
            "success": True,
            "url": self.page.url,
            "title": await self.page.title(),
            "content": content[:5000],
        }

    async def execute_js(self, script: str) -> dict:
        try:
            result = await self.page.evaluate(script)
            return {"success": True, "result": str(result)}
        except Exception as e:
            return {"success": False, "error": str(e)}

    async def new_tab(self, url: str = None) -> dict:
        context = self.page.context
        new_page = await context.new_page()
        self.page = new_page
        if url:
            await self.navigate(url)
        return {"success": True, "action": "new tab opened"}

    async def go_back(self) -> dict:
        await self.page.go_back()
        return {"success": True, "url": self.page.url}

    async def wait_for_element(self, selector: str, timeout: int = 10000) -> dict:
        try:
            await self.page.wait_for_selector(selector, timeout=timeout)
            return {"success": True, "found": True}
        except:
            return {"success": False, "found": False}


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    async def test():
        b = BrowserHands(headless=True)
        await b.start()
        r = await b.navigate("https://www.google.com")
        print("OK BrowserHands:", r["title"])
        await b.stop()

    asyncio.run(test())
