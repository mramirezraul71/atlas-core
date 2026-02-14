"""Agente Navegador: AtlasNavigator. Órgano sensorial externo: Chromium headless + BeautifulSoup + captura a temp_vision/."""
from __future__ import annotations

import asyncio
import logging
import os
import re
import time
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger("atlas.navigator")

BASE = Path(__file__).resolve().parent.parent
TEMP_VISION = BASE / "temp_vision"
FETCH_TIMEOUT_MS = 15000


def _clean_visible_text(html: str) -> str:
    try:
        from bs4 import BeautifulSoup
        soup = BeautifulSoup(html or "", "html.parser")
        for tag in soup(["script", "style", "nav", "footer", "header"]):
            tag.decompose()
        text = soup.get_text(separator="\n")
        text = re.sub(r"\n{3,}", "\n\n", text)
        return text.strip()
    except Exception:
        return (html or "").strip()


class AtlasNavigator:
    """Navegador asíncrono: Chromium headless, extracción de texto con BeautifulSoup, screenshot en temp_vision/."""

    def __init__(self, headless: bool = True, timeout_ms: int = FETCH_TIMEOUT_MS) -> None:
        self._headless = headless
        self._timeout_ms = timeout_ms
        self._playwright = None
        self._browser = None

    async def start_browser(self) -> None:
        """Inicia contexto Chromium en modo headless."""
        try:
            from playwright.async_api import async_playwright
            self._playwright = await async_playwright().start()
            self._browser = await self._playwright.chromium.launch(headless=self._headless)
        except Exception as e:
            logger.exception("start_browser: %s", e)
            raise

    async def fetch_page_content(self, url: str) -> Dict[str, Any]:
        """
        Visita URL, wait_until=networkidle, extrae título y texto visible (BeautifulSoup),
        captura a temp_vision/. Retorna dict con status, title, text, screenshot_path;
        si falla: {"status": "error", "message": "..."}.
        """
        TEMP_VISION.mkdir(parents=True, exist_ok=True)
        default_fail = {"status": "error", "message": "unknown", "title": "", "text": "", "screenshot_path": ""}
        if not self._browser:
            try:
                await self.start_browser()
            except Exception as e:
                return {"status": "error", "message": str(e), "title": "", "text": "", "screenshot_path": ""}
        try:
            page = await self._browser.new_page()
            try:
                await page.goto(url, wait_until="networkidle", timeout=self._timeout_ms)
            except Exception as e:
                return {
                    "status": "error",
                    "message": str(e),
                    "title": "",
                    "text": "",
                    "screenshot_path": "",
                }
            try:
                title = await page.title() or ""
                html = await page.content()
                text = _clean_visible_text(html)
            except Exception as e:
                title = ""
                text = ""
                logger.debug("extract content: %s", e)
            screenshot_path = ""
            try:
                name = f"capture_{int(time.time())}_{os.path.basename(url.strip('/') or 'page')}.png"
                name = re.sub(r"[^\w\-.]", "_", name)[:80]
                path = TEMP_VISION / name
                await page.screenshot(path=str(path))
                screenshot_path = str(path)
            except Exception as e:
                logger.debug("screenshot: %s", e)
            await page.close()
            return {
                "status": "ok",
                "message": "",
                "title": title,
                "text": text,
                "screenshot_path": screenshot_path,
            }
        except asyncio.TimeoutError as e:
            return {"status": "error", "message": f"timeout: {e}", "title": "", "text": "", "screenshot_path": ""}
        except Exception as e:
            logger.debug("fetch_page_content: %s", e)
            return {"status": "error", "message": str(e), "title": "", "text": "", "screenshot_path": ""}

    async def close_browser(self) -> None:
        """Cierra navegador y Playwright."""
        try:
            if self._browser:
                await self._browser.close()
                self._browser = None
        except Exception as e:
            logger.debug("close browser: %s", e)
        try:
            if self._playwright:
                await self._playwright.stop()
                self._playwright = None
        except Exception as e:
            logger.debug("stop playwright: %s", e)


async def fetch_page_text(url: str) -> Optional[str]:
    """Compatibilidad con Orquestador: extrae solo texto. Tolerante a fallos."""
    nav = AtlasNavigator()
    try:
        await nav.start_browser()
        out = await nav.fetch_page_content(url)
        await nav.close_browser()
        if out.get("status") == "ok":
            return out.get("text") or ""
        return None
    except Exception as e:
        logger.debug("fetch_page_text: %s", e)
        return None


if __name__ == "__main__":
    async def _test() -> None:
        nav = AtlasNavigator(headless=True, timeout_ms=15000)
        try:
            await nav.start_browser()
            url = "https://news.ycombinator.com/"
            print(f"Visitando {url} ...")
            result = await nav.fetch_page_content(url)
            await nav.close_browser()
            if result.get("status") == "ok":
                title = result.get("title", "")
                text = result.get("text", "")
                path = result.get("screenshot_path", "")
                print(f"Título: {title}")
                print(f"Texto (resumen, primeros 800 chars):\n{text[:800]}...")
                print(f"Captura guardada: {path}")
            else:
                print(f"Error: {result.get('message', '')}")
        finally:
            await nav.close_browser()

    asyncio.run(_test())
