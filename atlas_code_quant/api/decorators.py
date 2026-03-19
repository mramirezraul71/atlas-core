"""API decorators for Quant routes."""
from __future__ import annotations

import inspect
from functools import wraps
from typing import Any, Awaitable, Callable, TypeVar, cast

from api.schemas import OrderRequest, StdResponse
from execution.account_manager import AccountManager
from execution.tradier_execution import should_route_to_tradier


F = TypeVar("F", bound=Callable[..., Awaitable[Any]])


def require_live_confirmation(func: F) -> F:
    """Block accidental live submits unless the caller explicitly confirms."""

    signature = inspect.signature(func)
    account_manager = AccountManager()

    @wraps(func)
    async def wrapper(*args: Any, **kwargs: Any) -> Any:
        bound = signature.bind_partial(*args, **kwargs)
        body = bound.arguments.get("body")
        if isinstance(body, OrderRequest) and not body.preview and should_route_to_tradier(body):
            try:
                _, session = account_manager.resolve(
                    account_scope=body.account_scope,
                    account_id=body.account_id,
                )
                if session.classification == "live" and not body.live_confirmed:
                    return StdResponse(
                        ok=False,
                        error="Live account requires explicit live_confirmed=true before execution",
                        data={
                            "account_session": session.to_dict(),
                            "confirmation_required": True,
                        },
                    )
            except Exception:
                return StdResponse(
                    ok=False,
                    error="Unable to confirm Tradier live/paper account safely",
                    data={"confirmation_required": True},
                )
        return await func(*args, **kwargs)

    wrapper.__signature__ = signature
    return cast(F, wrapper)
