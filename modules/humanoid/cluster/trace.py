"""Correlation ID propagation for cluster requests."""
from __future__ import annotations

import contextvars
import uuid
from typing import Optional

_correlation_id: contextvars.ContextVar[Optional[str]] = contextvars.ContextVar("cluster_correlation_id", default=None)


def get_correlation_id() -> Optional[str]:
    return _correlation_id.get()


def set_correlation_id(cid: Optional[str]) -> None:
    if cid:
        _correlation_id.set(cid)
    else:
        try:
            _correlation_id.set(None)
        except LookupError:
            pass


def new_correlation_id() -> str:
    cid = str(uuid.uuid4())
    _correlation_id.set(cid)
    return cid
