"""Tests F4 — LEAN parser tolerante a versión."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from atlas_code_quant.lean.parser import (
    fitness_from_statistics,
    parse_equity_curve,
    parse_orders,
    parse_statistics,
)


# ── statistics ────────────────────────────────────────────────────────────
def test_parse_statistics_missing_returns_empty(tmp_path: Path) -> None:
    assert parse_statistics(tmp_path / "no.json") == {}


def test_parse_statistics_invalid_json_returns_empty(tmp_path: Path) -> None:
    p = tmp_path / "bad.json"
    p.write_text("not-json", encoding="utf-8")
    assert parse_statistics(p) == {}


def test_parse_statistics_reads_dict(tmp_path: Path) -> None:
    p = tmp_path / "stats.json"
    p.write_text(json.dumps({"Sharpe Ratio": "1.4", "Total Return": 0.21}), encoding="utf-8")
    data = parse_statistics(p)
    assert data["Sharpe Ratio"] == "1.4"


def test_fitness_from_statistics_normalizes_keys() -> None:
    fit = fitness_from_statistics({
        "Sharpe Ratio": "1.4",
        "Total Return": "0.21",
        "Drawdown": "0.08",
        "Win Rate": "0.55",
        "Profit Factor": "1.7",
        "Total Trades": "23",
    })
    assert fit["sharpe"] == pytest.approx(1.4)
    assert fit["total_return"] == pytest.approx(0.21)
    assert fit["drawdown"] == pytest.approx(0.08)
    assert fit["trades"] == pytest.approx(23.0)


def test_fitness_from_statistics_empty_input() -> None:
    assert fitness_from_statistics({}) == {}


# ── orders ────────────────────────────────────────────────────────────────
def test_parse_orders_missing_returns_empty(tmp_path: Path) -> None:
    assert parse_orders(tmp_path / "no.json") == []


def test_parse_orders_list_of_dicts(tmp_path: Path) -> None:
    p = tmp_path / "orders.json"
    p.write_text(json.dumps([
        {"Id": "1", "Symbol": "spy", "Direction": "Buy", "Quantity": 10, "Price": 520.5,
         "Time": "2026-04-26T20:00:00Z", "Status": "Filled", "Type": "Market"},
        {"id": "2", "symbol": "QQQ", "side": "sell", "quantity": 5, "price": 460.1,
         "time": "2026-04-26T20:01:00Z"},
    ]), encoding="utf-8")
    orders = parse_orders(p)
    assert len(orders) == 2
    assert orders[0].symbol == "SPY"
    assert orders[0].side == "buy"
    assert orders[0].quantity == 10.0
    assert orders[1].symbol == "QQQ"
    assert orders[1].side == "sell"


def test_parse_orders_wrapped_object(tmp_path: Path) -> None:
    p = tmp_path / "orders.json"
    p.write_text(json.dumps({"Orders": [{"Id": "1", "Symbol": "AAPL", "Quantity": 1, "Price": 200.0}]}),
                 encoding="utf-8")
    orders = parse_orders(p)
    assert len(orders) == 1
    assert orders[0].symbol == "AAPL"


# ── equity ────────────────────────────────────────────────────────────────
def test_parse_equity_curve_missing_returns_empty(tmp_path: Path) -> None:
    assert parse_equity_curve(tmp_path / "nofile.csv") == []


def test_parse_equity_curve_with_header(tmp_path: Path) -> None:
    p = tmp_path / "equity.csv"
    p.write_text(
        "timestamp,equity\n"
        "2026-04-25T20:00:00Z,25000.00\n"
        "2026-04-25T20:01:00Z,25012.50\n",
        encoding="utf-8",
    )
    pts = parse_equity_curve(p)
    assert len(pts) == 2
    assert pts[0].equity == pytest.approx(25000.0)
    assert pts[-1].equity == pytest.approx(25012.5)


def test_parse_equity_curve_no_header(tmp_path: Path) -> None:
    p = tmp_path / "equity.csv"
    p.write_text("2026-04-25T20:00:00Z,25000.00\n", encoding="utf-8")
    pts = parse_equity_curve(p)
    assert len(pts) == 1
