from __future__ import annotations

from atlas_scanner.config_loader import ScanConfig
from atlas_scanner.fixtures.offline import OFFLINE_DEFAULT_SYMBOLS, OFFLINE_EXTENDED_SYMBOLS
from atlas_scanner.models import SymbolSnapshot


def select_offline_universe(
    config: ScanConfig,
    snapshots: tuple[SymbolSnapshot, ...],
) -> tuple[SymbolSnapshot, ...]:
    if config.universe_name == "default":
        allowed = set(OFFLINE_DEFAULT_SYMBOLS)
        return tuple(snapshot for snapshot in snapshots if snapshot.symbol in allowed)
    if config.universe_name == "extended":
        allowed = set(OFFLINE_EXTENDED_SYMBOLS)
        return tuple(snapshot for snapshot in snapshots if snapshot.symbol in allowed)
    return snapshots

