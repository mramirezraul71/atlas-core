"""Replay protection: nonce store with TTL. Reject reuse."""
from __future__ import annotations

import os
import time
from typing import Dict

_NONCES: Dict[str, float] = {}


def _ttl_sec() -> int:
    try:
        return int(os.getenv("APPROVALS_REPLAY_TTL_SECONDS", "3600") or 3600)
    except (TypeError, ValueError):
        return 3600


def consume_nonce(nonce: str) -> bool:
    """
    Consume nonce. Returns True if valid (first use), False if reused or invalid.
    """
    if not nonce or not nonce.strip():
        return False
    key = nonce.strip()
    now = time.time()
    ttl = _ttl_sec()
    # Purge expired
    expired = [k for k, t in _NONCES.items() if now - t > ttl]
    for k in expired:
        _NONCES.pop(k, None)
    if key in _NONCES:
        return False  # Replay
    _NONCES[key] = now
    return True
