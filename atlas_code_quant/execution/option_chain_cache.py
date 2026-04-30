"""Cache de cadenas de opciones con TTL — Fase 1.

Evita llamadas repetidas a Tradier durante el ciclo de 5s del LiveLoop.
TTL configurable por env ATLAS_OPTION_CHAIN_TTL_SEC (default 60s).
Pure class — sin threads, sin estado global.
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field

logger = logging.getLogger("atlas.execution.option_chain_cache")

_DEFAULT_TTL = int(os.getenv("ATLAS_OPTION_CHAIN_TTL_SEC", "60"))


@dataclass
class _CachedChain:
    symbol:      str
    expiration:  str
    fetched_at:  float
    data:        dict


class OptionChainCache:
    """Cache LRU-simple de cadenas de opciones Tradier.

    Uso::
        cache = OptionChainCache(ttl_sec=60)
        chain = cache.get_or_fetch("AAPL", "2025-02-21", tradier_client, "paper")
    """

    def __init__(self, ttl_sec: int = _DEFAULT_TTL, max_entries: int = 50) -> None:
        self._ttl         = ttl_sec
        self._max_entries = max_entries
        self._store: dict[str, _CachedChain] = {}

    # ── API pública ───────────────────────────────────────────────────────────

    def get_or_fetch(
        self,
        symbol:          str,
        expiration:      str,            # "YYYY-MM-DD"
        tradier_client,                  # TradierClient-like con método get_option_chain
        scope:           str = "paper",
    ) -> dict:
        """Retorna cadena desde cache o la descarga y cachea."""
        key = self._key(symbol, expiration)
        cached = self._store.get(key)

        if cached and self._is_fresh(cached):
            logger.debug("CHAIN CACHE HIT %s exp=%s", symbol, expiration)
            return cached.data

        logger.debug("CHAIN CACHE MISS %s exp=%s — fetching Tradier", symbol, expiration)
        try:
            data = tradier_client.get_option_chain(symbol, expiration, scope=scope)
        except Exception as exc:
            logger.warning("Error fetching option chain %s/%s: %s", symbol, expiration, exc)
            return cached.data if cached else {}

        self._store[key] = _CachedChain(
            symbol=symbol, expiration=expiration,
            fetched_at=time.time(), data=data,
        )
        self._evict_if_needed()
        return data

    def get_expirations(
        self,
        symbol:         str,
        tradier_client,
        scope:          str = "paper",
    ) -> list[str]:
        """Lista de fechas de expiración disponibles para el símbolo."""
        exp_key = f"_exp_{symbol}"
        cached = self._store.get(exp_key)
        if cached and self._is_fresh(cached):
            return cached.data.get("expirations", [])

        try:
            expirations = tradier_client.get_option_expirations(symbol, scope=scope)
        except Exception as exc:
            logger.warning("Error fetching expirations %s: %s", symbol, exc)
            return []

        self._store[exp_key] = _CachedChain(
            symbol=symbol, expiration="",
            fetched_at=time.time(),
            data={"expirations": expirations},
        )
        return expirations

    def invalidate(self, symbol: str) -> None:
        """Elimina todas las entradas del símbolo del cache."""
        keys = [k for k in self._store if k.startswith(f"{symbol.upper()}|") or
                k == f"_exp_{symbol.upper()}"]
        for k in keys:
            self._store.pop(k, None)

    def clear(self) -> None:
        self._store.clear()

    def stats(self) -> dict:
        now = time.time()
        fresh   = sum(1 for c in self._store.values() if self._is_fresh(c))
        expired = len(self._store) - fresh
        return {"total": len(self._store), "fresh": fresh, "expired": expired}

    # ── Internos ──────────────────────────────────────────────────────────────

    def _key(self, symbol: str, expiration: str) -> str:
        return f"{symbol.upper()}|{expiration}"

    def _is_fresh(self, cached: _CachedChain) -> bool:
        return (time.time() - cached.fetched_at) < self._ttl

    def _evict_if_needed(self) -> None:
        if len(self._store) <= self._max_entries:
            return
        # Eliminar el más antiguo
        oldest_key = min(self._store, key=lambda k: self._store[k].fetched_at)
        self._store.pop(oldest_key, None)
