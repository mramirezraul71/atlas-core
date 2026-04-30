"""
Demo mínima: abrir Bull Put Spread mock → mark → cerrar.
Ejecutar desde la raíz del paquete padre:
  python -m atlas_options_brain.simulator.paper_demo
"""
from __future__ import annotations

from datetime import datetime, timezone
from dataclasses import replace

from atlas_options_brain.dsl.strategy import BearCallSpread, BullPutSpread
from atlas_options_brain.models.option_contract import OptionsChain
from atlas_options_brain.simulator.paper import PaperSimulator, rebuild_contracts_from_chain

# Reutiliza helpers del test mock (misma cadena sintética)
from atlas_options_brain.tests.test_dsl_mock import make_chain


def _bump_mids(legs, delta: float):
    """Devuelve contratos alineados al orden de legs con mid desplazado."""
    out = []
    for leg in legs:
        c = leg.contract
        nb = max(0.01, round(c.bid + delta, 2))
        na = max(0.02, round(c.ask + delta, 2))
        out.append(replace(c, bid=nb, ask=na, last_price=round((nb + na) / 2, 4)))
    return out


def _print_history_summary(pos):
    print(f"Historial MTM en memoria ({pos.snapshot_count} snapshots):")
    for i, s in enumerate(pos.snapshots):
        kinds = ""
        if s.match_info:
            kinds = " match_kinds=[" + ",".join(m.match_kind.value for m in s.match_info) + "]"
        print(
            f"  [{i}] {s.timestamp.isoformat()} "
            f"uPnL={s.unrealized_pnl} mids={s.leg_mids}{kinds}"
        )


def main():
    chain = make_chain()
    sim = PaperSimulator()
    strat_bps = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
    strat_bcs = BearCallSpread.from_chain(chain, short_delta=0.25, width=10)
    pos = sim.open_position(strat_bps, opened_at=datetime(2025, 6, 1, 15, 0, tzinfo=timezone.utc))
    pos2 = sim.open_position(strat_bcs, opened_at=datetime(2025, 6, 1, 15, 5, tzinfo=timezone.utc))
    print(
        f"Abiertas en simulador: {[p.position_id for p in sim.list_open_positions()]} "
        f"(BPS entry_net={pos.entry_net_premium}, BCS entry_net={pos2.entry_net_premium})"
    )

    cur = _bump_mids(pos.entry_legs, -0.02)
    sim.snapshot(pos, cur, record=False)
    print(f"MTM (sin guardar) unrealized={pos.unrealized_pnl(cur)} mids={tuple(c.mid for c in cur)}")

    t1 = datetime(2025, 6, 1, 15, 30, tzinfo=timezone.utc)
    chain_m1 = OptionsChain(
        symbol=chain.symbol,
        expiration=chain.expiration,
        spot_price=chain.spot_price,
        contracts=_bump_mids(pos.entry_legs, -0.01),
    )
    pos.mark_with_chain(chain_m1, at=t1, record=True)

    t2 = datetime(2025, 6, 1, 16, 0, tzinfo=timezone.utc)
    chain_m2 = OptionsChain(
        symbol=chain.symbol,
        expiration=chain.expiration,
        spot_price=chain.spot_price + 0.5,
        contracts=_bump_mids(pos.entry_legs, 0.04),
    )
    snap_chain = pos.mark_with_chain(chain_m2, at=t2, record=True)
    print(
        f"Último mark_with_chain unrealized={snap_chain.unrealized_pnl} "
        f"mids={snap_chain.leg_mids} entry_net={snap_chain.entry_net_premium}"
    )

    t3 = datetime(2025, 6, 1, 16, 30, tzinfo=timezone.utc)
    cur3 = _bump_mids(pos.entry_legs, 0.02)
    sim.snapshot(pos, cur3, at=t3, record=True)

    contracts_diag, infos_diag = rebuild_contracts_from_chain(
        pos, chain_m2, with_diagnostics=True
    )
    assert snap_chain.match_info == infos_diag
    print("Diagnóstico por pata (último mark_with_chain guardado, chain t2):")
    for leg, c_now, mi in zip(pos.entry_legs, contracts_diag, infos_diag):
        print(
            f"  leg {mi.leg_index}: underlying={leg.contract.symbol} "
            f"strike_apertura={leg.contract.strike} strike_mtm={c_now.strike} "
            f"kind={mi.match_kind.value} "
            f"contract_symbol_open={mi.original_contract_symbol!r} "
            f"contract_symbol_mtm={mi.used_contract_symbol!r}"
        )

    _print_history_summary(pos)

    cur2 = _bump_mids(pos.entry_legs, 0.03)
    pnl = sim.close_position(pos, cur2, closed_at=datetime(2025, 6, 2, 15, 0, tzinfo=timezone.utc))
    print(
        f"Cerrado {pos.position_id} realized_pnl={pnl} exit_liq={pos.exit_liquidation_net}"
    )
    print(
        f"Registro tras cierre: abiertas={[p.position_id for p in sim.list_open_positions()]} "
        f"cerradas={[p.position_id for p in sim.list_closed_positions()]}"
    )
    assert sim.get_position("pos-0001") is pos and not pos.is_open
    assert sim.get_position("pos-0002") is pos2 and pos2.is_open


if __name__ == "__main__":
    main()
