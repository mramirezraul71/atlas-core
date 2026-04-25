from __future__ import annotations

from .base import BaseProvider


class YFinanceFallbackProvider(BaseProvider):
    provider_name: str = "yfinance_fallback"

