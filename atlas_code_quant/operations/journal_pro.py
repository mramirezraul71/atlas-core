"""High-level journal facade for operational dashboards."""
from __future__ import annotations

from typing import Any

from journal.service import TradingJournalService


class JournalProService:
    def __init__(self, journal: TradingJournalService | None = None) -> None:
        self.journal = journal or TradingJournalService()

    def snapshot(self, *, limit: int = 5) -> dict[str, Any]:
        stats = self.journal.stats()
        entries = self.journal.entries(limit=limit)
        return {
            "generated_at": stats.get("generated_at"),
            "stats": stats,
            "recent_entries": entries.get("items", []),
            "recent_entries_count": entries.get("count", 0),
        }

