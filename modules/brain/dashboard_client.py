"""AtlasDashboardClient: HTTP async (aiohttp) al Dashboard de Push. Exponential backoff, state read para evitar duplicados."""
from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Any, Dict, Optional

logger = logging.getLogger("atlas.dashboard")

DASHBOARD_BASE = (os.getenv("ATLAS_DASHBOARD_URL") or "http://127.0.0.1:8791").rstrip("/")
BACKOFF_INIT = float(os.getenv("ATLAS_DASHBOARD_BACKOFF_INIT", "1.0"))
BACKOFF_MAX = float(os.getenv("ATLAS_DASHBOARD_BACKOFF_MAX", "60.0"))
BACKOFF_MULT = float(os.getenv("ATLAS_DASHBOARD_BACKOFF_MULT", "2.0"))


class AtlasDashboardClient:
    def __init__(self) -> None:
        self._base = DASHBOARD_BASE
        self._session: Any = None
        self._backoff = BACKOFF_INIT

    async def _session_ensure(self) -> bool:
        try:
            import aiohttp
            if self._session is None or self._session.closed:
                self._session = aiohttp.ClientSession()
            return True
        except ImportError:
            logger.warning("aiohttp no instalado")
            return False

    async def get_state(self) -> Optional[Dict]:
        for _ in range(3):
            try:
                if not await self._session_ensure():
                    await asyncio.sleep(self._backoff)
                    continue
                import aiohttp
                async with self._session.get(f"{self._base}/api/push/state", timeout=aiohttp.ClientTimeout(total=5)) as r:
                    if r.status == 200:
                        self._backoff = BACKOFF_INIT
                        return await r.json()
            except Exception as e:
                logger.debug("get_state fail: %s", e)
                self._backoff = min(self._backoff * BACKOFF_MULT, BACKOFF_MAX)
                await asyncio.sleep(self._backoff)
        return None

    async def send_command(self, target: str = "NEXUS_ARM", action: str = "update_state", value: Any = 1) -> bool:
        payload = {"target": target, "action": action, "value": value}
        for _ in range(3):
            try:
                if not await self._session_ensure():
                    await asyncio.sleep(self._backoff)
                    continue
                import aiohttp
                async with self._session.post(
                    f"{self._base}/api/push/command",
                    json=payload,
                    timeout=aiohttp.ClientTimeout(total=5),
                ) as r:
                    if r.status == 200:
                        self._backoff = BACKOFF_INIT
                        return True
            except Exception as e:
                logger.debug("send_command fail: %s", e)
                self._backoff = min(self._backoff * BACKOFF_MULT, BACKOFF_MAX)
                await asyncio.sleep(self._backoff)
        return False

    async def close(self) -> None:
        try:
            if self._session and not self._session.closed:
                await self._session.close()
        except Exception:
            pass
        self._session = None
