"""
lotto_quant.dashboard.links
===========================

Cross-link helpers between the Lotto-Quant Streamlit HUD and the Atlas
central V4 dashboard (the FastAPI/PUSH adapter served at `/ui`).

These are tiny, dependency-free utilities — kept out of `hud.py` so they
can be unit-tested without importing Streamlit.

Environment variables
---------------------
ATLAS_DASHBOARD_URL  Base URL of the Atlas central dashboard.
                     Default: http://127.0.0.1:8791/ui
                     The `/ui` suffix is appended automatically if missing
                     so users can also set just `http://host:port`.

ATLAS_LOTTO_QUANT_URL  Base URL of the Lotto-Quant Streamlit HUD, used by
                       the V4 adapter to redirect users back here.
                       Default: http://127.0.0.1:8501
"""

from __future__ import annotations

import os

DEFAULT_CENTRAL_URL = "http://127.0.0.1:8791/ui"
DEFAULT_LOTTO_URL = "http://127.0.0.1:8501"


def atlas_central_url() -> str:
    """Return the canonical URL for the Atlas central V4 dashboard."""
    raw = (os.getenv("ATLAS_DASHBOARD_URL") or DEFAULT_CENTRAL_URL).strip()
    raw = raw.rstrip("/")
    if not raw:
        raw = DEFAULT_CENTRAL_URL
    if not raw.endswith("/ui"):
        raw = f"{raw}/ui"
    return raw


def atlas_lotto_url() -> str:
    """Return the canonical URL for the Atlas Lotto-Quant Streamlit HUD."""
    raw = (os.getenv("ATLAS_LOTTO_QUANT_URL") or DEFAULT_LOTTO_URL).strip()
    raw = raw.rstrip("/")
    if not raw:
        raw = DEFAULT_LOTTO_URL
    return raw
