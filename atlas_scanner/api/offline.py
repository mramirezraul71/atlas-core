from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.config_loader import ScanConfig, build_default_scan_config_offline
from atlas_scanner.runner.offline import OfflineScanResult, run_offline_scan
from atlas_scanner.scoring.offline import ScoredSymbol


@dataclass
class OfflineScanner:
    """
    High-level offline scanner facade.

    Orquesta config, runner y scoring para ofrecer una API estable
    y extensible a otros modulos de atlas-core.
    """

    def scan(self, config: ScanConfig | None = None, **filters: object) -> OfflineScanResult:
        effective_config = config or build_default_scan_config_offline()
        return run_offline_scan(effective_config, **filters)

    def scan_top(
        self,
        top_n: int,
        config: ScanConfig | None = None,
        **filters: object,
    ) -> tuple[ScoredSymbol, ...]:
        if top_n <= 0:
            return ()
        result = self.scan(config=config, **filters)
        return result.ranked_symbols[:top_n]

    def scan_and_explain(
        self,
        top_n: int | None = None,
        config: ScanConfig | None = None,
        **filters: object,
    ) -> OfflineScanResult:
        result = self.scan(config=config, **filters)
        if top_n is None:
            return result
        if top_n <= 0:
            trimmed_ranked: tuple[ScoredSymbol, ...] = ()
            trimmed_candidates = ()
        else:
            trimmed_ranked = result.ranked_symbols[:top_n]
            trimmed_candidates = result.candidate_opportunities[:top_n]

        next_meta = dict(result.meta)
        next_meta["top_n"] = top_n
        next_meta["total_ranked_symbols"] = len(trimmed_ranked)
        return OfflineScanResult(
            config=result.config,
            reference_datetime=result.reference_datetime,
            selected_symbols=result.selected_symbols,
            ranked_symbols=trimmed_ranked,
            universe_name=result.universe_name,
            data_source_path=result.data_source_path,
            meta=next_meta,
            candidate_opportunities=trimmed_candidates,
        )


_default_scanner = OfflineScanner()


def scan_offline(config: ScanConfig | None = None, **filters: object) -> OfflineScanResult:
    return _default_scanner.scan(config=config, **filters)


def scan_offline_top_n(
    top_n: int,
    config: ScanConfig | None = None,
    **filters: object,
) -> tuple[ScoredSymbol, ...]:
    return _default_scanner.scan_top(top_n=top_n, config=config, **filters)


def scan_offline_and_explain(
    top_n: int | None = None,
    config: ScanConfig | None = None,
    **filters: object,
) -> OfflineScanResult:
    return _default_scanner.scan_and_explain(top_n=top_n, config=config, **filters)

