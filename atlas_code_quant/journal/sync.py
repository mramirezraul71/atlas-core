"""Background synchronization loop for the trading journal."""
from __future__ import annotations

import asyncio
import logging

from config.settings import settings
from journal.service import TradingJournalService

logger = logging.getLogger("quant.journal.sync")


class JournalSyncService:
    def __init__(self, journal: TradingJournalService) -> None:
        self.journal = journal
        self._task: asyncio.Task | None = None
        self._running = False

    async def start(self) -> None:
        if self._task and not self._task.done():
            return
        self._running = True
        self.journal.init_db()
        self._task = asyncio.create_task(self._run_loop(), name="atlas-quant-journal-sync")

    async def stop(self) -> None:
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def run_once(self) -> dict[str, object]:
        results: list[dict[str, object]] = []
        for scope, token in (("paper", settings.tradier_paper_token), ("live", settings.tradier_live_token)):
            if not token:
                continue
            try:
                outcome = await asyncio.to_thread(self.journal.sync_scope, scope)
                results.append(outcome)
            except Exception:
                logger.exception("Journal sync failed for scope %s", scope)
                results.append({"scope": scope, "error": "sync_failed"})
        return {"results": results}

    async def _run_loop(self) -> None:
        while self._running:
            try:
                await self.run_once()
            except Exception:
                logger.exception("Journal sync loop failed")
            await asyncio.sleep(max(settings.tradier_journal_sync_interval_sec, 2))
