"""Atlas Radar — Universo determinista de símbolos para barrido multi-símbolo (F5).

Este módulo provee un **universo curado, determinista y sin dependencias de red**
para el motor de oportunidades multi-símbolo del Radar institucional. Es la
fuente de verdad por defecto de :mod:`atlas_adapter.services.radar_batch_engine`.

Diseño (F5):

* No hay llamadas HTTP, ni a base de datos, ni a Quant. La lista vive en el
  código y se construye una sola vez por proceso. Esto permite:
    - Tests deterministas y rápidos.
    - Que el Radar mantenga un comportamiento honesto incluso si Quant /
      atlas_scanner / proveedores externos están caídos.
* Categorías iniciales:
    - ``etf``: ETFs de alta liquidez (índice amplio + sectores SPDR).
    - ``index``: índices spot referencia (no operables directamente, marcados
      ``optionable=False`` para evitar promesas operativas).
    - ``equity``: equities optionables curadas (mega-cap líquido).
* Cada símbolo declara: ``asset_class``, ``sector`` (o ``None``),
  ``optionable`` (bool), ``enabled`` (bool, default True).
* Filtros soportados (todos componen AND):
    - ``asset_class`` (str)
    - ``sector`` (str, exact match case-insensitive)
    - ``optionable`` (bool)
    - ``enabled_only`` (bool, default True)
    - ``symbols`` (subset explícito)
    - ``limit`` (int, recorte determinista al final)
* El orden de salida es **estable**: respeta el orden de declaración interno y
  re-ordena sólo cuando se filtra por ``symbols`` siguiendo el orden recibido.

Reglas duras (F5):
    * NO ejecuta trading.
    * NO consulta proveedores externos.
    * NO impone ``ATLAS_RADAR_MULTI_SYMBOL_ENABLED`` como condicional; es
      flag documental.

Ver también:
    * docs/ATLAS_RADAR_F5_MULTI_SYMBOL_OPPORTUNITIES.md
    * atlas_adapter/services/radar_batch_engine.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

__all__ = [
    "UniverseEntry",
    "UniverseProvider",
    "DEFAULT_UNIVERSE",
    "default_universe_provider",
]


@dataclass(frozen=True)
class UniverseEntry:
    """Entrada inmutable del universo."""

    symbol: str
    name: str
    asset_class: str  # "etf" | "index" | "equity"
    sector: str | None = None
    optionable: bool = True
    enabled: bool = True
    tags: tuple[str, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, object]:
        return {
            "symbol": self.symbol,
            "name": self.name,
            "asset_class": self.asset_class,
            "sector": self.sector,
            "optionable": self.optionable,
            "enabled": self.enabled,
            "tags": list(self.tags),
        }


# ---------------------------------------------------------------------------
# Universo curado por defecto. Determinista: no se altera entre procesos.
# ---------------------------------------------------------------------------

_ETFS: tuple[UniverseEntry, ...] = (
    UniverseEntry("SPY", "SPDR S&P 500 ETF Trust", "etf", "broad_market", True, True, ("liquid", "core")),
    UniverseEntry("QQQ", "Invesco QQQ Trust", "etf", "broad_market", True, True, ("liquid", "core")),
    UniverseEntry("IWM", "iShares Russell 2000 ETF", "etf", "broad_market", True, True, ("liquid",)),
    UniverseEntry("DIA", "SPDR Dow Jones Industrial Average ETF", "etf", "broad_market", True, True, ("liquid",)),
    UniverseEntry("XLK", "Technology Select Sector SPDR", "etf", "technology", True, True, ()),
    UniverseEntry("XLF", "Financial Select Sector SPDR", "etf", "financials", True, True, ()),
    UniverseEntry("XLE", "Energy Select Sector SPDR", "etf", "energy", True, True, ()),
    UniverseEntry("XLV", "Health Care Select Sector SPDR", "etf", "healthcare", True, True, ()),
    UniverseEntry("XLI", "Industrial Select Sector SPDR", "etf", "industrials", True, True, ()),
    UniverseEntry("XLY", "Consumer Discretionary Select Sector SPDR", "etf", "consumer_discretionary", True, True, ()),
    UniverseEntry("XLP", "Consumer Staples Select Sector SPDR", "etf", "consumer_staples", True, True, ()),
    UniverseEntry("XLU", "Utilities Select Sector SPDR", "etf", "utilities", True, True, ()),
    UniverseEntry("XLB", "Materials Select Sector SPDR", "etf", "materials", True, True, ()),
    UniverseEntry("XLRE", "Real Estate Select Sector SPDR", "etf", "real_estate", True, True, ()),
    UniverseEntry("XLC", "Communication Services Select Sector SPDR", "etf", "communication_services", True, True, ()),
)

_INDICES: tuple[UniverseEntry, ...] = (
    UniverseEntry("SPX", "S&P 500 Index", "index", "broad_market", False, True, ("reference",)),
    UniverseEntry("NDX", "Nasdaq-100 Index", "index", "broad_market", False, True, ("reference",)),
    UniverseEntry("RUT", "Russell 2000 Index", "index", "broad_market", False, True, ("reference",)),
    UniverseEntry("VIX", "CBOE Volatility Index", "index", "volatility", False, True, ("reference", "volatility")),
)

_EQUITIES: tuple[UniverseEntry, ...] = (
    UniverseEntry("AAPL", "Apple Inc.", "equity", "technology", True, True, ("megacap",)),
    UniverseEntry("MSFT", "Microsoft Corp.", "equity", "technology", True, True, ("megacap",)),
    UniverseEntry("NVDA", "NVIDIA Corp.", "equity", "technology", True, True, ("megacap",)),
    UniverseEntry("AMZN", "Amazon.com Inc.", "equity", "consumer_discretionary", True, True, ("megacap",)),
    UniverseEntry("GOOGL", "Alphabet Inc. Class A", "equity", "communication_services", True, True, ("megacap",)),
    UniverseEntry("META", "Meta Platforms Inc.", "equity", "communication_services", True, True, ("megacap",)),
    UniverseEntry("TSLA", "Tesla Inc.", "equity", "consumer_discretionary", True, True, ("megacap",)),
    UniverseEntry("AMD", "Advanced Micro Devices", "equity", "technology", True, True, ()),
    UniverseEntry("AVGO", "Broadcom Inc.", "equity", "technology", True, True, ()),
    UniverseEntry("NFLX", "Netflix Inc.", "equity", "communication_services", True, True, ()),
    UniverseEntry("JPM", "JPMorgan Chase & Co.", "equity", "financials", True, True, ()),
    UniverseEntry("BAC", "Bank of America Corp.", "equity", "financials", True, True, ()),
    UniverseEntry("XOM", "Exxon Mobil Corp.", "equity", "energy", True, True, ()),
    UniverseEntry("CVX", "Chevron Corp.", "equity", "energy", True, True, ()),
    UniverseEntry("UNH", "UnitedHealth Group Inc.", "equity", "healthcare", True, True, ()),
    UniverseEntry("LLY", "Eli Lilly and Co.", "equity", "healthcare", True, True, ()),
    UniverseEntry("WMT", "Walmart Inc.", "equity", "consumer_staples", True, True, ()),
    UniverseEntry("COST", "Costco Wholesale Corp.", "equity", "consumer_staples", True, True, ()),
    UniverseEntry("BA", "Boeing Co.", "equity", "industrials", True, True, ()),
    UniverseEntry("CAT", "Caterpillar Inc.", "equity", "industrials", True, True, ()),
)

DEFAULT_UNIVERSE: tuple[UniverseEntry, ...] = tuple(_ETFS + _INDICES + _EQUITIES)


# ---------------------------------------------------------------------------
# Provider
# ---------------------------------------------------------------------------


class UniverseProvider:
    """Provee el universo curado del Radar con filtros deterministas.

    El provider no hace red ni I/O. Es seguro instanciarlo como singleton.
    """

    def __init__(self, entries: Iterable[UniverseEntry] | None = None) -> None:
        if entries is None:
            self._entries: tuple[UniverseEntry, ...] = DEFAULT_UNIVERSE
        else:
            self._entries = tuple(entries)
        # Index por símbolo (case-insensitive) para lookup O(1).
        self._by_symbol: dict[str, UniverseEntry] = {e.symbol.upper(): e for e in self._entries}

    # -- Lookup directo --------------------------------------------------

    def get(self, symbol: str) -> UniverseEntry | None:
        if not symbol:
            return None
        return self._by_symbol.get(symbol.strip().upper())

    def all(self) -> tuple[UniverseEntry, ...]:
        return self._entries

    def asset_classes(self) -> tuple[str, ...]:
        seen: list[str] = []
        for e in self._entries:
            if e.asset_class not in seen:
                seen.append(e.asset_class)
        return tuple(seen)

    def sectors(self) -> tuple[str, ...]:
        seen: list[str] = []
        for e in self._entries:
            if e.sector and e.sector not in seen:
                seen.append(e.sector)
        return tuple(seen)

    # -- Listado filtrado -----------------------------------------------

    def list(
        self,
        *,
        limit: int | None = None,
        asset_class: str | None = None,
        sector: str | None = None,
        optionable: bool | None = None,
        enabled_only: bool = True,
        symbols: Iterable[str] | None = None,
    ) -> list[UniverseEntry]:
        """Retorna entradas filtradas, manteniendo orden estable.

        Filtros componen con AND. ``symbols`` (si se provee) restringe a ese
        subconjunto y respeta el orden recibido. ``limit`` se aplica al final.
        """
        ac = (asset_class or "").strip().lower() or None
        sec = (sector or "").strip().lower() or None

        if symbols is not None:
            wanted = [s.strip().upper() for s in symbols if s and str(s).strip()]
            base: list[UniverseEntry] = []
            for sym in wanted:
                ent = self._by_symbol.get(sym)
                if ent is not None:
                    base.append(ent)
        else:
            base = list(self._entries)

        out: list[UniverseEntry] = []
        for e in base:
            if enabled_only and not e.enabled:
                continue
            if ac is not None and e.asset_class.lower() != ac:
                continue
            if sec is not None and (e.sector or "").lower() != sec:
                continue
            if optionable is not None and bool(e.optionable) != bool(optionable):
                continue
            out.append(e)

        if limit is not None:
            try:
                lim = int(limit)
            except (TypeError, ValueError):
                lim = 0
            if lim > 0:
                out = out[:lim]
        return out

    def symbols(
        self,
        *,
        limit: int | None = None,
        asset_class: str | None = None,
        sector: str | None = None,
        optionable: bool | None = None,
        enabled_only: bool = True,
    ) -> list[str]:
        return [
            e.symbol
            for e in self.list(
                limit=limit,
                asset_class=asset_class,
                sector=sector,
                optionable=optionable,
                enabled_only=enabled_only,
            )
        ]


# Singleton por defecto: no introduce estado mutable global.
_DEFAULT_PROVIDER: UniverseProvider | None = None


def default_universe_provider() -> UniverseProvider:
    """Singleton lazy del provider con el universo curado por defecto."""
    global _DEFAULT_PROVIDER
    if _DEFAULT_PROVIDER is None:
        _DEFAULT_PROVIDER = UniverseProvider()
    return _DEFAULT_PROVIDER
