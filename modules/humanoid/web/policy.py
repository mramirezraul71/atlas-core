"""Web module policy: allowlist of domains and rate limit."""
from __future__ import annotations

import os
import re
import time
from collections import defaultdict
from typing import Dict, List, Optional

RATE_LIMIT_REQUESTS = 30
RATE_LIMIT_WINDOW_SEC = 60


def _allowed_domains() -> List[str]:
    v = os.getenv("WEB_ALLOWED_DOMAINS", "")
    if not v.strip():
        return []  # empty = allow all for now, or deny all
    return [x.strip().lower() for x in v.split(",") if x.strip()]


def _domain_allowed(url: str) -> bool:
    domains = _allowed_domains()
    if not domains:
        return True
    try:
        from urllib.parse import urlparse
        host = urlparse(url).netloc.lower().split(":")[0]
        return host in domains or any(host.endswith("." + d) for d in domains)
    except Exception:
        return False


_request_times: Dict[str, List[float]] = defaultdict(list)


def check_rate_limit(key: str = "default") -> bool:
    """True if under limit."""
    now = time.time()
    window = RATE_LIMIT_WINDOW_SEC
    times = _request_times[key]
    times.append(now)
    while times and times[0] < now - window:
        times.pop(0)
    return len(times) <= RATE_LIMIT_REQUESTS


def can_navigate(url: str) -> tuple[bool, Optional[str]]:
    """(allowed, reason)."""
    if not _domain_allowed(url):
        return False, "domain not in WEB_ALLOWED_DOMAINS"
    if not check_rate_limit("navigate"):
        return False, "rate limit exceeded"
    return True, None
