from __future__ import annotations

from dataclasses import dataclass, field

from .symbol_snapshot import SymbolSnapshot


@dataclass(frozen=True)
class ScanSnapshot:
    snapshot_id: str
    created_at: str
    universe_name: str
    symbols: tuple[SymbolSnapshot, ...]
    config_version: str
    meta: dict[str, object] = field(default_factory=dict)

