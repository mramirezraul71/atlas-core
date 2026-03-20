"""Universe catalog for the opportunity scanner."""
from __future__ import annotations

import json
import logging
import re
import time
from io import StringIO
from pathlib import Path
from typing import Any
from urllib.request import Request, urlopen

import pandas as pd


logger = logging.getLogger("quant.scanner.universe")

NASDAQ_LISTED_URL = "https://www.nasdaqtrader.com/dynamic/SymDir/nasdaqlisted.txt"
OTHER_LISTED_URL = "https://www.nasdaqtrader.com/dynamic/SymDir/otherlisted.txt"
_USER_AGENT = "ATLAS-CodeQuant/1.0"
_SYMBOL_RE = re.compile(r"^[A-Z][A-Z.-]{0,9}$")
_EXCLUDED_NAME_TOKENS = (
    "WARRANT",
    "RIGHT",
    "UNIT",
    "UNITS",
    "PREFERRED",
    "PREF ",
    " DEPOSITARY",
    " DEPOSITORY",
    "NOTE",
    "NOTES",
)


def _normalize_symbol(value: str) -> str:
    return str(value or "").strip().upper().replace(".", "-")


def _looks_tradeable(symbol: str, security_name: str) -> bool:
    if not symbol or not _SYMBOL_RE.match(symbol):
        return False
    name_upper = security_name.upper()
    return not any(token in name_upper for token in _EXCLUDED_NAME_TOKENS)


class ScannerUniverseCatalog:
    """Loads and caches a broad US equities universe for rotating scans."""

    def __init__(self, cache_path: Path, cache_ttl_sec: int = 86_400) -> None:
        self.cache_path = Path(cache_path)
        self.cache_ttl_sec = max(3600, int(cache_ttl_sec))

    def get_us_equities(self, force_refresh: bool = False) -> dict[str, Any]:
        cached = self._load_cache()
        if cached and not force_refresh:
            age_sec = max(0.0, time.time() - float(cached.get("fetched_epoch", 0.0)))
            if age_sec <= self.cache_ttl_sec and cached.get("symbols"):
                return cached

        fresh = self._fetch_us_equities()
        self._save_cache(fresh)
        return fresh

    def _load_cache(self) -> dict[str, Any] | None:
        if not self.cache_path.exists():
            return None
        try:
            data = json.loads(self.cache_path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else None
        except Exception as exc:
            logger.warning("No se pudo leer cache de universo: %s", exc)
            return None

    def _save_cache(self, payload: dict[str, Any]) -> None:
        self.cache_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _download_text(self, url: str) -> str:
        request = Request(url, headers={"User-Agent": _USER_AGENT})
        with urlopen(request, timeout=20) as response:
            return response.read().decode("utf-8", errors="ignore")

    def _fetch_us_equities(self) -> dict[str, Any]:
        nasdaq = self._parse_table(self._download_text(NASDAQ_LISTED_URL), listed_kind="nasdaq")
        other = self._parse_table(self._download_text(OTHER_LISTED_URL), listed_kind="other")
        rows = nasdaq + other
        deduped: dict[str, dict[str, Any]] = {}
        for item in rows:
            deduped.setdefault(item["symbol"], item)
        symbols = sorted(deduped)
        payload = {
            "kind": "us_equities",
            "source": "nasdaq_trader_symbol_directory",
            "source_urls": [NASDAQ_LISTED_URL, OTHER_LISTED_URL],
            "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "fetched_epoch": time.time(),
            "total_symbols": len(symbols),
            "symbols": symbols,
            "meta": {
                "nasdaq_rows": len(nasdaq),
                "other_rows": len(other),
                "deduped_rows": len(symbols),
            },
        }
        logger.info("Catalogo de universo actualizado con %s simbolos", len(symbols))
        return payload

    def _parse_table(self, raw_text: str, listed_kind: str) -> list[dict[str, Any]]:
        df = pd.read_csv(StringIO(raw_text), sep="|")
        if df.empty:
            return []

        first_col = str(df.columns[0])
        footer_mask = df[first_col].astype(str).str.contains("File Creation Time", na=False)
        df = df.loc[~footer_mask].copy()

        if listed_kind == "nasdaq":
            symbol_col = "Symbol"
            name_col = "Security Name"
            test_col = "Test Issue"
            etf_col = "ETF"
            exchange = "NASDAQ"
        else:
            symbol_col = "ACT Symbol"
            name_col = "Security Name"
            test_col = "Test Issue"
            etf_col = "ETF"
            exchange = "OTHER"

        rows: list[dict[str, Any]] = []
        for row in df.to_dict("records"):
            raw_symbol = row.get(symbol_col)
            security_name = str(row.get(name_col) or "").strip()
            is_test = str(row.get(test_col) or "").strip().upper() == "Y"
            if is_test:
                continue
            symbol = _normalize_symbol(str(raw_symbol or ""))
            if not _looks_tradeable(symbol, security_name):
                continue
            rows.append(
                {
                    "symbol": symbol,
                    "security_name": security_name,
                    "is_etf": str(row.get(etf_col) or "").strip().upper() == "Y",
                    "exchange": exchange,
                }
            )
        return rows
