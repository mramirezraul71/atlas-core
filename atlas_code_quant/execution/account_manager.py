"""Tradier account management helpers for Code-Quant."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from backtesting.winning_probability import TradierClient, TradierScope
from execution.tradier_controls import TradierAccountSession, check_pdt_status, resolve_account_session


@dataclass
class AccountStatusSnapshot:
    session: TradierAccountSession
    pdt_status: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "account_session": self.session.to_dict(),
            "pdt_status": self.pdt_status,
            "classification": self.session.classification,
            "days_trades_used": min(int(self.pdt_status.get("day_trades_last_window") or 0), 3),
        }


class AccountManager:
    """Resolve and classify Tradier accounts from the configured credentials/base URLs."""

    def resolve(
        self,
        *,
        account_scope: TradierScope | None = None,
        account_id: str | None = None,
        tradier_token: str | None = None,
        tradier_base_url: str | None = None,
        force_refresh: bool = False,
    ) -> tuple[TradierClient, TradierAccountSession]:
        return resolve_account_session(
            account_scope=account_scope,
            account_id=account_id,
            tradier_token=tradier_token,
            tradier_base_url=tradier_base_url,
            force_refresh=force_refresh,
        )

    def status(
        self,
        *,
        account_scope: TradierScope | None = None,
        account_id: str | None = None,
        tradier_token: str | None = None,
        tradier_base_url: str | None = None,
        force_refresh: bool = False,
    ) -> AccountStatusSnapshot:
        client, session = self.resolve(
            account_scope=account_scope,
            account_id=account_id,
            tradier_token=tradier_token,
            tradier_base_url=tradier_base_url,
            force_refresh=force_refresh,
        )
        pdt_status = check_pdt_status(client, session)
        return AccountStatusSnapshot(session=session, pdt_status=pdt_status)
