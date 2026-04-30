#!/usr/bin/env python3
"""
Experimento de humo (funcionalidad) del módulo atlas_radar_kalshi.

No requiere NEXUS/PUSH ni :8791. Comprueba imports por dominio, arbitraje,
KPIs y risk. Opcional: preflight HTTP (con red) vía --network.

Uso (desde la raíz del worktree):
  set PYTHONPATH=.
  python scripts/radar_kalshi_smoke.py
  python scripts/radar_kalshi_smoke.py --network
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import tempfile
from pathlib import Path


def _root() -> Path:
    return Path(__file__).resolve().parents[1]


def main() -> int:
    root = _root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    ap = argparse.ArgumentParser(description="Atlas Radar Kalshi — smoke / experimento")
    ap.add_argument(
        "--network",
        action="store_true",
        help="Ejecutar además preflight (requiere conectividad; puede fallar sin credenciales).",
    )
    args = ap.parse_args()
    print(f"[radar_smoke] root={root}")

    errors: list[str] = []

    # 1) Imports dominio
    try:
        import modules.atlas_radar_kalshi as radar  # noqa: F401
        from modules.atlas_radar_kalshi.arbitrage_engine import ArbitrageEngine
        from modules.atlas_radar_kalshi.canonical import market_from_payload
        from modules.atlas_radar_kalshi.control.calibration_kpis import compute_calibration_kpis
        from modules.atlas_radar_kalshi.execution import reconcile_kalshi_from_router
        from modules.atlas_radar_kalshi.execution_router import ExecutionModeRouter
        from modules.atlas_radar_kalshi.ingestion import venue_of_event
        from modules.atlas_radar_kalshi.risk_engine import RiskEngine, RiskLimits, RiskState
        from modules.atlas_radar_kalshi.scanner import MarketEvent
        from modules.atlas_radar_kalshi.state.journal import Journal
    except Exception as exc:
        print("FAIL: imports", exc)
        return 2

    # 2) market_from_payload
    m = market_from_payload("Test market?", "2026-12-01T00:00:00Z", kalshi="KX-T")
    assert m.key, "canonical key"
    print(f"[radar_smoke] canonical key ok: {m.key[:48]}...")

    # 3) venue_of_event
    ev = MarketEvent(
        kind="ticker", market_ticker="POLY:1", payload={"source": "polymarket"}
    )
    assert venue_of_event(ev) == "polymarket"
    print("[radar_smoke] venue_of_event OK")

    # 4) Arbitraje cross-venue
    eng = ArbitrageEngine(min_profit=0.01)
    eng.upsert_quote(
        ticker="KX-1",
        source="kalshi",
        title="Same question text",
        close_time="2026-06-15T12:00:00Z",
        yes_mid=0.55,
        yes_ask=0.56,
        no_ask=0.45,
    )
    eng.upsert_quote(
        ticker="POLY:99",
        source="polymarket",
        title="Same question text",
        close_time="2026-06-15T12:00:00Z",
        yes_mid=0.30,
    )
    rows = eng.detect(top_n=10)
    if not rows:
        errors.append("arbitrage_detect_empty")
    else:
        print(f"[radar_smoke] arbitraje: {len(rows)} oportunidades, top={rows[0].get('strategy')}")

    # 5) Risk + sizing
    re = RiskEngine(
        RiskLimits(
            kelly_fraction=0.5,
            max_position_pct=0.05,
            max_kalshi_venue_exposure_pct=0.5,
            max_polymarket_venue_exposure_pct=0.5,
        ),
        state=RiskState(balance_cents=1_000_000),
    )
    st = re.status()
    assert "exposure_by_venue" in st
    print("[radar_smoke] risk.status OK (incl. exposure_by_venue)")

    # 6) Reconcile router (paper)
    r = ExecutionModeRouter()
    rep = asyncio.run(reconcile_kalshi_from_router(r))
    if not isinstance(rep, dict):
        errors.append("reconcile_not_dict")
    print(
        f"[radar_smoke] reconcile: "
        f"{ list(rep.keys()) if isinstance(rep, dict) else rep }"
    )

    # 7) calibration KPIs (journal tmp)
    with tempfile.TemporaryDirectory() as td:
        j = Journal(Path(td))
        j.write(
            "decisions",
            {
                "ticker": "KX-S",
                "readout": {"p_ensemble": 0.65},
                "gate": {"side": "YES"},
            },
        )
        j.write("exits", {"ticker": "KX-S", "pnl_cents": 10, "entry": 50})
        ck = compute_calibration_kpis(j)
        print(
            f"[radar_smoke] calibration_kpis: n={ck.n_pairs} brier={ck.brier_score} gap={ck.ev_gap_cents}"
        )

    # 8) Preflight opcional (red)
    if args.network:
        from modules.atlas_radar_kalshi.config import get_settings
        from modules.atlas_radar_kalshi.preflight import run_preflight

        os.environ.setdefault("ATLAS_LOG_DIR", str(root / "logs" / "radar_smoke"))
        get_settings.cache_clear()  # type: ignore[attr-defined]
        out = asyncio.run(run_preflight(get_settings()))
        print(
            f"[radar_smoke] preflight ok field={out.get('ok')} readiness={out.get('readiness', {})}"
        )

    if errors:
        print("WARN:", errors)
        return 1
    print("[radar_smoke] RESULTADO: OK (smoke completo).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
