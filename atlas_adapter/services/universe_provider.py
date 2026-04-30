"""Proveedor de universo optionable multi-símbolo para Radar F2 (PUSH).

Reutiliza datos estáticos del paquete ``atlas_code_quant.scanner`` cuando el
import es posible; si falla (entorno mínimo, rutas), aplica un **fallback curado
local** documentado — sin llamadas externas ni Tradier.

No establece dependencia de runtime del **scanner en ejecución**; solo catálogos.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from atlas_code_quant.config.feature_flags import AtlasFeatureFlags

logger = logging.getLogger("atlas.radar.universe")

# Fallback mínimo si no se pueden importar catálogos del scanner (misma intención
# que ETF/index universe, sin duplicar cientos de filas).
_FALLBACK_ETF = (
    "SPY",
    "QQQ",
    "IWM",
    "DIA",
    "XLF",
    "XLE",
    "XLK",
    "GLD",
    "TLT",
    "VXX",
)
_FALLBACK_INDEX = ("SPX", "NDX", "RUT", "VIX", "XSP")
# Subconjunto líquido y habitual en cadena de opciones (no sustituye due diligence).
_FALLBACK_EQUITIES = ("AAPL", "MSFT", "NVDA", "AMZN", "GOOGL", "META", "TSLA")

try:
    from atlas_code_quant.scanner.etf_universe import ETF_OPTIONS_UNIVERSE
    from atlas_code_quant.scanner.index_universe import INDEX_PROFILES
except Exception as exc:  # pragma: no cover - entornos parciales
    logger.info(
        "universe_provider: import scanner catalogs fallback (reason=%s). "
        "Usando listas curadas locales.",
        exc.__class__.__name__,
    )
    ETF_OPTIONS_UNIVERSE = {}
    INDEX_PROFILES = {}


@dataclass(frozen=True, slots=True)
class UniverseEntry:
    """Activo optionable curado para el batch Radar."""

    symbol: str
    asset_class: str  # equity_etf | index_option | equity_stock
    sector: str
    optionable: bool = True
    notes: str = ""


class UniverseProvider:
    """Cache TTL configurable vía ``ATLAS_RADAR_UNIVERSE_REFRESH_SEC``."""

    def __init__(self, flags: AtlasFeatureFlags | None = None) -> None:
        self._flags = flags or AtlasFeatureFlags()
        self._cached: list[UniverseEntry] | None = None
        self._cached_at: float = 0.0

    def _ttl_sec(self) -> float:
        return float(self._flags.radar_universe_refresh_sec)

    def _build_static_universe(self) -> list[UniverseEntry]:
        entries: list[UniverseEntry] = []
        seen: set[str] = set()

        def add(e: UniverseEntry) -> None:
            s = e.symbol.strip().upper()
            if not s or s in seen:
                return
            seen.add(s)
            entries.append(
                UniverseEntry(
                    symbol=s,
                    asset_class=e.asset_class,
                    sector=e.sector,
                    optionable=e.optionable,
                    notes=e.notes,
                )
            )

        if ETF_OPTIONS_UNIVERSE:
            for sym, prof in ETF_OPTIONS_UNIVERSE.items():
                add(
                    UniverseEntry(
                        symbol=sym,
                        asset_class="equity_etf",
                        sector=str(getattr(prof, "sector", "unknown")),
                        notes=str(getattr(prof, "name", ""))[:120],
                    )
                )
        else:
            for sym in _FALLBACK_ETF:
                add(
                    UniverseEntry(
                        symbol=sym,
                        asset_class="equity_etf",
                        sector="fallback_etf",
                        notes="fallback_catalog",
                    )
                )

        if INDEX_PROFILES:
            for root, prof in INDEX_PROFILES.items():
                add(
                    UniverseEntry(
                        symbol=str(getattr(prof, "root", root)),
                        asset_class="index_option",
                        sector="index",
                        notes=str(getattr(prof, "display_name", ""))[:120],
                    )
                )
        else:
            for sym in _FALLBACK_INDEX:
                add(
                    UniverseEntry(
                        symbol=sym,
                        asset_class="index_option",
                        sector="index",
                        notes="fallback_index",
                    )
                )

        for sym in _FALLBACK_EQUITIES:
            add(
                UniverseEntry(
                    symbol=sym,
                    asset_class="equity_stock",
                    sector="liquid_large_cap",
                    notes="curated_equity",
                )
            )

        entries.sort(key=lambda x: (x.asset_class, x.symbol))
        return entries

    def refresh(self, *, force: bool = False) -> list[UniverseEntry]:
        """Reconstruye el universo en memoria (honesto ante TTL)."""
        now = time.monotonic()
        if (
            not force
            and self._cached is not None
            and (now - self._cached_at) < self._ttl_sec()
        ):
            return list(self._cached)

        self._cached = self._build_static_universe()
        self._cached_at = now
        logger.info(
            "universe_provider: refresh entries=%s ttl_s=%s",
            len(self._cached),
            self._ttl_sec(),
        )
        return list(self._cached)

    def get_optionable_universe(self, *, max_size: int | None = None) -> list[UniverseEntry]:
        """Lista optionable; ``max_size`` acota el lote (p. ej. flag batch)."""
        u = self.refresh()
        if max_size is None:
            max_size = self._flags.radar_max_symbols_per_batch
        max_size = max(1, int(max_size))
        return u[:max_size]

    def search(self, query: str, *, limit: int = 25) -> list[UniverseEntry]:
        """Prefijo insensible a mayúsculas sobre símbolo (sin motor Quant)."""
        q = (query or "").strip().upper()
        lim = max(1, min(int(limit or 25), 200))
        if not q:
            return self.get_optionable_universe(max_size=lim)
        out: list[UniverseEntry] = []
        for e in self.refresh():
            if e.symbol.startswith(q):
                out.append(e)
                if len(out) >= lim:
                    break
        return out
