"""Tests del simulador paper — solo datos mock, sin red (stdlib unittest)."""
from __future__ import annotations

import math
import unittest
from datetime import datetime, timezone
from dataclasses import replace

from atlas_options_brain.dsl.strategy import BullPutSpread, BullCallSpread, BearCallSpread
from atlas_options_brain.models.option_contract import OptionType, OptionsChain
from atlas_options_brain.simulator.paper import (
    LegMatchInfo,
    MatchKind,
    PaperSimulator,
    Position,
    _clone_leg,
    rebuild_contracts_from_chain,
)
from atlas_options_brain.tests.test_dsl_mock import make_chain


def _strike_eq(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=1e-6)


def _contracts_for_mark(legs, bid_delta: float, ask_delta: float):
    out = []
    for leg in legs:
        c = leg.contract
        nb = max(0.01, round(c.bid + bid_delta, 2))
        na = max(0.02, round(c.ask + ask_delta, 2))
        if na <= nb:
            na = round(nb + 0.01, 2)
        out.append(
            replace(
                c,
                bid=nb,
                ask=na,
                last_price=round((nb + na) / 2, 4),
            )
        )
    return out


class TestPaperSimulator(unittest.TestCase):
    def test_open_bull_put_entry_matches_strategy_net(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat, opened_at=datetime(2025, 1, 1, tzinfo=timezone.utc))
        self.assertTrue(pos.is_open)
        self.assertEqual(pos.entry_net_premium, strat.net_premium)
        self.assertEqual(len(pos.entry_legs), 2)

    def test_unrealized_zero_if_same_mids(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        cur = [replace(leg.contract) for leg in pos.entry_legs]
        self.assertEqual(pos.unrealized_pnl(cur), 0.0)

    def test_close_sets_realized_and_blocks_double_close(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        cur = _contracts_for_mark(pos.entry_legs, 0.05, 0.05)
        pnl = sim.close_position(pos, cur)
        self.assertEqual(pos.realized_pnl, pnl)
        self.assertIsNotNone(pos.exit_liquidation_net)
        self.assertFalse(pos.is_open)
        with self.assertRaises(RuntimeError):
            pos.close(cur)

    def test_bull_call_debit_unrealized_moves_with_mids(self):
        chain = make_chain()
        strat = BullCallSpread.from_chain(chain, long_delta=0.5, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        self.assertLess(pos.entry_net_premium, 0)
        cur_up = _contracts_for_mark(pos.entry_legs, 0.5, 0.5)
        u = pos.unrealized_pnl(cur_up)
        self.assertIsInstance(u, float)

    def test_snapshot_contains_mids(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        cur = _contracts_for_mark(pos.entry_legs, -0.01, -0.01)
        snap = sim.snapshot(pos, cur, at=datetime(2025, 2, 1, tzinfo=timezone.utc))
        self.assertEqual(snap.entry_net_premium, pos.entry_net_premium)
        self.assertEqual(len(snap.leg_mids), 2)
        self.assertIsNone(snap.match_info)

    def test_rebuild_without_diagnostics_returns_list_same_as_tuple_first(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=[
                replace(c, bid=round(c.bid + 0.02, 2), ask=round(c.ask + 0.02, 2))
                for c in chain.contracts
            ],
        )
        plain = rebuild_contracts_from_chain(pos, chain2)
        both = rebuild_contracts_from_chain(pos, chain2, with_diagnostics=True)
        self.assertIsInstance(plain, list)
        self.assertIsInstance(both, tuple)
        contracts, infos = both
        self.assertEqual(plain, contracts)
        self.assertEqual(len(infos), 2)
        self.assertIsInstance(infos[0], LegMatchInfo)

    def test_rebuild_diagnostics_all_exact(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=[
                replace(c, bid=round(c.bid + 0.05, 2), ask=round(c.ask + 0.05, 2))
                for c in chain.contracts
            ],
        )
        _, infos = rebuild_contracts_from_chain(
            pos, chain2, with_diagnostics=True
        )
        self.assertEqual(
            [i.match_kind for i in infos],
            [MatchKind.EXACT, MatchKind.EXACT],
        )

    def test_rebuild_diagnostics_near_mixed_exact_and_near(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        short_ref = pos.entry_legs[1].contract
        near_strike = short_ref.strike + 0.5
        near_contract = replace(
            short_ref,
            strike=near_strike,
            bid=round(short_ref.bid + 0.12, 2),
            ask=round(short_ref.ask + 0.12, 2),
            last_price=round(short_ref.last_price + 0.12, 4),
            volume=777,
        )
        filtered = [
            c
            for c in chain.contracts
            if not (
                c.option_type == OptionType.PUT
                and _strike_eq(c.strike, short_ref.strike)
                and c.expiration == short_ref.expiration
            )
        ]
        chain_near = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=filtered + [near_contract],
        )
        _, infos = rebuild_contracts_from_chain(
            pos, chain_near, strike_tolerance=1.0, with_diagnostics=True
        )
        kinds = [i.match_kind for i in infos]
        self.assertIn(MatchKind.EXACT, kinds)
        self.assertIn(MatchKind.NEAR, kinds)
        self.assertNotIn(MatchKind.FROZEN, kinds)

    def test_rebuild_diagnostics_tiny_tolerance_frozen_when_no_exact(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        short_ref = pos.entry_legs[1].contract
        near_strike = short_ref.strike + 0.5
        near_contract = replace(short_ref, strike=near_strike, volume=100)
        filtered = [
            c
            for c in chain.contracts
            if not (
                c.option_type == OptionType.PUT
                and _strike_eq(c.strike, short_ref.strike)
                and c.expiration == short_ref.expiration
            )
        ]
        chain_near = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=filtered + [near_contract],
        )
        _, infos = rebuild_contracts_from_chain(
            pos, chain_near, strike_tolerance=0.0, with_diagnostics=True
        )
        self.assertEqual(infos[1].match_kind, MatchKind.FROZEN)
        self.assertEqual(infos[0].match_kind, MatchKind.EXACT)

    def test_rebuild_diagnostics_empty_chain_all_frozen(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        empty = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=[],
        )
        _, infos = rebuild_contracts_from_chain(
            pos, empty, with_diagnostics=True
        )
        self.assertEqual(
            [i.match_kind for i in infos],
            [MatchKind.FROZEN, MatchKind.FROZEN],
        )

    def test_rebuild_exact_match_returns_updated_quotes(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        bumped = [
            replace(
                c,
                bid=round(c.bid + 0.07, 2),
                ask=round(c.ask + 0.07, 2),
                last_price=round(c.last_price + 0.07, 4),
            )
            for c in chain.contracts
        ]
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=bumped,
        )
        rebuilt = rebuild_contracts_from_chain(pos, chain2)
        self.assertEqual(len(rebuilt), 2)
        for leg, c in zip(pos.entry_legs, rebuilt, strict=True):
            self.assertTrue(_strike_eq(c.strike, leg.contract.strike))
            self.assertEqual(c.option_type, leg.contract.option_type)
            self.assertGreater(c.mid, leg.contract.mid)

    def test_rebuild_duplicate_strike_picks_higher_volume(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        ref = pos.entry_legs[0].contract
        dup_a = replace(ref, volume=100, contract_symbol="DUP_A", bid=0.05, ask=0.10)
        dup_b = replace(ref, volume=800, contract_symbol="DUP_B", bid=0.06, ask=0.11)
        filtered = [
            c
            for c in chain.contracts
            if not (
                c.option_type == OptionType.PUT
                and _strike_eq(c.strike, ref.strike)
                and c.expiration == ref.expiration
            )
        ]
        chain_dups = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=filtered + [dup_a, dup_b],
        )
        rebuilt = rebuild_contracts_from_chain(pos, chain_dups)
        self.assertEqual(rebuilt[0].volume, 800)
        self.assertEqual(rebuilt[0].contract_symbol, "DUP_B")

    def test_rebuild_near_strike_within_tolerance(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        short_ref = pos.entry_legs[1].contract
        near_strike = short_ref.strike + 0.5
        near_contract = replace(
            short_ref,
            strike=near_strike,
            bid=round(short_ref.bid + 0.12, 2),
            ask=round(short_ref.ask + 0.12, 2),
            last_price=round(short_ref.last_price + 0.12, 4),
            volume=777,
        )
        filtered = [
            c
            for c in chain.contracts
            if not (
                c.option_type == OptionType.PUT
                and _strike_eq(c.strike, short_ref.strike)
                and c.expiration == short_ref.expiration
            )
        ]
        chain_near = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=filtered + [near_contract],
        )
        rebuilt = rebuild_contracts_from_chain(
            pos, chain_near, strike_tolerance=1.0
        )
        self.assertTrue(_strike_eq(rebuilt[1].strike, near_strike))
        self.assertEqual(rebuilt[1].volume, 777)

    def test_rebuild_no_candidate_falls_back_to_frozen_leg(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        long_ref = pos.entry_legs[0].contract
        # Sin el strike del long ni vecinos dentro de 1.0 en la cadena sintética.
        filtered = [
            c
            for c in chain.contracts
            if not (
                c.option_type == OptionType.PUT
                and abs(c.strike - long_ref.strike) <= 1.0 + 1e-9
                and c.expiration == long_ref.expiration
            )
        ]
        chain_gap = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=filtered,
        )
        rebuilt = rebuild_contracts_from_chain(
            pos, chain_gap, strike_tolerance=1.0
        )
        self.assertTrue(_strike_eq(rebuilt[0].strike, long_ref.strike))
        self.assertEqual(rebuilt[0].bid, long_ref.bid)
        self.assertEqual(rebuilt[0].ask, long_ref.ask)

    def test_mark_with_chain_matches_manual_rebuild(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=[
                replace(c, bid=round(c.bid - 0.03, 2), ask=round(c.ask - 0.03, 2))
                for c in chain.contracts
            ],
        )
        snap = pos.mark_with_chain(chain2, at=datetime(2025, 3, 1, tzinfo=timezone.utc))
        manual = rebuild_contracts_from_chain(pos, chain2)
        self.assertEqual(snap.unrealized_pnl, pos.unrealized_pnl(manual))
        self.assertEqual(snap.leg_mids, tuple(c.mid for c in manual))
        self.assertEqual(snap.timestamp.year, 2025)
        self.assertIsNotNone(snap.match_info)
        assert snap.match_info is not None
        self.assertEqual(
            [m.match_kind for m in snap.match_info],
            [MatchKind.EXACT, MatchKind.EXACT],
        )
        _, diag = rebuild_contracts_from_chain(pos, chain2, with_diagnostics=True)
        self.assertEqual(snap.match_info, diag)
        self.assertEqual(pos.snapshot_count, 0)

    def test_mark_with_chain_match_info_coherent_with_rebuild(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        chain_mark = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=[
                replace(c, bid=round(c.bid + 0.01, 2), ask=round(c.ask + 0.01, 2))
                for c in chain.contracts
            ],
        )
        snap = pos.mark_with_chain(chain_mark)
        _, expected = rebuild_contracts_from_chain(
            pos, chain_mark, with_diagnostics=True
        )
        self.assertEqual(snap.match_info, expected)


class TestSnapshotHistory(unittest.TestCase):
    """Historial MTM en memoria; política conservadora post-cierre."""

    def test_simulator_snapshot_record_false_leaves_history_empty(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        cur = _contracts_for_mark(pos.entry_legs, 0.01, 0.01)
        sim.snapshot(pos, cur, record=False)
        self.assertEqual(pos.snapshot_count, 0)
        self.assertIsNone(pos.last_snapshot)

    def test_simulator_snapshot_record_true_appends_one(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        cur = _contracts_for_mark(pos.entry_legs, 0.02, 0.02)
        t = datetime(2025, 4, 1, 14, 0, tzinfo=timezone.utc)
        snap = sim.snapshot(pos, cur, at=t, record=True)
        self.assertEqual(pos.snapshot_count, 1)
        self.assertIs(pos.last_snapshot, snap)
        self.assertEqual(pos.snapshots[0].timestamp, t)

    def test_multiple_snapshots_preserve_order(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        t0 = datetime(2025, 4, 1, 10, 0, tzinfo=timezone.utc)
        t1 = datetime(2025, 4, 1, 11, 0, tzinfo=timezone.utc)
        t2 = datetime(2025, 4, 1, 12, 0, tzinfo=timezone.utc)
        sim.snapshot(
            pos,
            _contracts_for_mark(pos.entry_legs, 0.01, 0.01),
            at=t0,
            record=True,
        )
        sim.snapshot(
            pos,
            _contracts_for_mark(pos.entry_legs, 0.02, 0.02),
            at=t1,
            record=True,
        )
        sim.snapshot(
            pos,
            _contracts_for_mark(pos.entry_legs, 0.03, 0.03),
            at=t2,
            record=True,
        )
        self.assertEqual([s.timestamp for s in pos.snapshots], [t0, t1, t2])

    def test_mark_with_chain_record_true_stores_match_info(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = PaperSimulator().open_position(strat)
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=[
                replace(c, bid=round(c.bid + 0.04, 2), ask=round(c.ask + 0.04, 2))
                for c in chain.contracts
            ],
        )
        pos.mark_with_chain(chain2, at=datetime(2025, 5, 1, tzinfo=timezone.utc), record=True)
        self.assertEqual(pos.snapshot_count, 1)
        stored = pos.snapshots[0]
        self.assertIsNotNone(stored.match_info)
        assert stored.match_info is not None
        self.assertEqual(len(stored.match_info), 2)

    def test_record_snapshot_raises_when_position_closed(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        sim.close_position(pos, _contracts_for_mark(pos.entry_legs, 0.0, 0.0))
        dummy = sim.snapshot(pos, _contracts_for_mark(pos.entry_legs, 0.01, 0.01))
        with self.assertRaises(RuntimeError):
            pos.record_snapshot(dummy)

    def test_mark_with_chain_record_true_raises_when_closed(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=list(chain.contracts),
        )
        sim.close_position(pos, _contracts_for_mark(pos.entry_legs, 0.0, 0.0))
        with self.assertRaises(RuntimeError):
            pos.mark_with_chain(chain2, record=True)

    def test_simulator_snapshot_record_true_raises_when_closed(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        sim.close_position(pos, _contracts_for_mark(pos.entry_legs, 0.0, 0.0))
        with self.assertRaises(RuntimeError):
            sim.snapshot(
                pos,
                _contracts_for_mark(pos.entry_legs, 0.01, 0.01),
                record=True,
            )

    def test_mark_without_record_when_closed_still_returns_snapshot(self):
        chain = make_chain()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        sim = PaperSimulator()
        pos = sim.open_position(strat)
        chain2 = OptionsChain(
            symbol=chain.symbol,
            expiration=chain.expiration,
            spot_price=chain.spot_price,
            contracts=list(chain.contracts),
        )
        sim.close_position(pos, _contracts_for_mark(pos.entry_legs, 0.0, 0.0))
        snap = pos.mark_with_chain(chain2, record=False)
        self.assertIsInstance(snap.unrealized_pnl, float)


class TestMultiPositionRegistry(unittest.TestCase):
    """Registro in-memory open/closed en PaperSimulator."""

    def test_open_two_positions_unique_ids_and_both_open(self):
        chain = make_chain()
        sim = PaperSimulator()
        p1 = sim.open_position(
            BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        )
        p2 = sim.open_position(
            BearCallSpread.from_chain(chain, short_delta=0.25, width=10)
        )
        self.assertEqual(p1.position_id, "pos-0001")
        self.assertEqual(p2.position_id, "pos-0002")
        self.assertNotEqual(p1.position_id, p2.position_id)
        self.assertEqual(len(sim.open_positions), 2)
        self.assertEqual(len(sim.list_open_positions()), 2)
        self.assertEqual(len(sim.closed_positions), 0)

    def test_close_moves_to_closed_and_removes_from_open(self):
        chain = make_chain()
        sim = PaperSimulator()
        p1 = sim.open_position(
            BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        )
        sim.open_position(
            BearCallSpread.from_chain(chain, short_delta=0.25, width=10)
        )
        cur = _contracts_for_mark(p1.entry_legs, 0.0, 0.0)
        sim.close_position(p1, cur)
        self.assertEqual(len(sim.open_positions), 1)
        self.assertEqual(len(sim.closed_positions), 1)
        self.assertFalse(p1.is_open)
        self.assertIs(sim.get_position("pos-0001"), p1)
        self.assertIsNotNone(sim.get_position("pos-0002"))
        self.assertTrue(sim.get_position("pos-0002").is_open)

    def test_get_position_none_for_unknown_id(self):
        sim = PaperSimulator()
        self.assertIsNone(sim.get_position("pos-0099"))

    def test_close_position_foreign_raises(self):
        chain = make_chain()
        sim_a = PaperSimulator()
        sim_b = PaperSimulator()
        pos = sim_a.open_position(
            BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        )
        cur = _contracts_for_mark(pos.entry_legs, 0.0, 0.0)
        with self.assertRaises(RuntimeError):
            sim_b.close_position(pos, cur)
        self.assertTrue(pos.is_open)
        self.assertIn(pos.position_id, sim_a.open_positions)

    def test_close_position_without_registry_raises(self):
        chain = make_chain()
        sim = PaperSimulator()
        strat = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        legs = tuple(_clone_leg(leg) for leg in strat.legs)
        orphan = Position(
            entry_legs=legs,
            opened_at=datetime.now(timezone.utc),
            entry_net_premium=round(sum(leg.premium for leg in legs), 2),
            position_id="",
        )
        cur = _contracts_for_mark(orphan.entry_legs, 0.0, 0.0)
        with self.assertRaises(RuntimeError):
            sim.close_position(orphan, cur)

    def test_ids_restart_per_simulator_instance(self):
        sim1 = PaperSimulator()
        sim2 = PaperSimulator()
        chain = make_chain()
        self.assertEqual(
            sim1.open_position(
                BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
            ).position_id,
            "pos-0001",
        )
        self.assertEqual(
            sim2.open_position(
                BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
            ).position_id,
            "pos-0001",
        )

    def test_double_close_still_fails_on_position(self):
        chain = make_chain()
        sim = PaperSimulator()
        pos = sim.open_position(
            BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        )
        cur = _contracts_for_mark(pos.entry_legs, 0.0, 0.0)
        sim.close_position(pos, cur)
        with self.assertRaises(RuntimeError):
            sim.close_position(pos, cur)


if __name__ == "__main__":
    unittest.main()
