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

    def position_management_snapshot(self, *, account_type: str | None = None, limit: int = 12) -> dict[str, Any]:
        return self.journal.position_management_snapshot(account_type=account_type, limit=limit)

    def exit_governance_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        return self.journal.exit_governance_snapshot(account_type=account_type, limit=limit)

    def post_trade_learning_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        return self.journal.post_trade_learning_snapshot(account_type=account_type, limit=limit)
