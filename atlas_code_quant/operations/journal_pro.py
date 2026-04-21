"""High-level journal facade for operational dashboards."""
from __future__ import annotations

from typing import Any

from journal.service import TradingJournalService
from local_models.integrations import (
    analyze_dashboard_screenshot,
    classify_journal_event,
    semantic_journal_search,
)


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

    def attribution_integrity_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        return self.journal.attribution_integrity_snapshot(account_type=account_type, limit=limit)

    def options_governance_adoption_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        return self.journal.options_governance_adoption_snapshot(account_type=account_type, limit=limit)

    def semantic_retrieval(self, query: str, *, limit: int = 5) -> dict[str, Any]:
        """Semantic retrieval over recent journal entries using local embeddings."""
        recent = self.journal.entries(limit=max(limit * 4, 24))
        items = list(recent.get("items", [])) if isinstance(recent, dict) else []
        return semantic_journal_search(query, items, top_k=limit)

    def lightweight_event_review(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Quick local review/classification for journal events."""
        return classify_journal_event(payload)

    def vision_screenshot_review(self, screenshot_path: str, *, prompt: str | None = None, premium: bool = False) -> dict[str, Any]:
        """Analyze dashboard screenshot with local vision model."""
        return analyze_dashboard_screenshot(screenshot_path, prompt=prompt, premium=premium)
