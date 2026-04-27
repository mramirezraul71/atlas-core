"""Tests F13 — LEAN adapter MVP (config + parser + runner).

Cubre:

* ``LeanAdapterConfig.load_config_from_env``: defaults OFF + parsing
  de variables de entorno + auto-selección mock/external.
* ``parse_statistics_payload`` / ``parse_orders_payload`` /
  ``parse_run_artifacts``: defensivos, soportan strings JSON, dicts
  anidados, % strings, payloads vacíos.
* ``run_backtest_for_strategy_intent``:
    - flag OFF → ``disabled`` con ``LEAN_DISABLED``.
    - intent inválido → ``LEAN_INVALID_INTENT``.
    - mock → determinismo (mismo intent ⇒ mismas métricas).
    - external feliz → lee statistics/orders y devuelve métricas.
    - external timeout / non-zero exit / binary missing → códigos
      de error honestos, sin colgar.
* AST guard: módulos LEAN F13 NO importan execution / autonomy /
  risk / vision / atlas_adapter / tradier / broker_router / live_*.
"""

from __future__ import annotations

import ast
import json
import os
import subprocess
from pathlib import Path
from unittest.mock import patch

import pytest

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.lean.config import (
    LeanAdapterConfig,
    load_config_from_env,
)
from atlas_code_quant.lean.parser.results import (
    LeanRunArtifacts,
    parse_orders_payload,
    parse_run_artifacts,
    parse_statistics_payload,
)
from atlas_code_quant.lean.runner.launcher import (
    ERROR_DISABLED,
    ERROR_INVALID_INTENT,
    ERROR_PROCESS_FAILED,
    ERROR_TIMEOUT,
    StrategyFitnessResult,
    run_backtest_for_strategy_intent,
)
from atlas_code_quant.strategies.factory.dispatch import (
    build_strategies_for_opportunity,
)
from atlas_code_quant.strategies.options.intent import StrategyIntent


def _make_intent() -> StrategyIntent:
    opp = RadarOpportunityInternal.from_payload(
        {
            "symbol": "SPY",
            "asset_class": "etf",
            "sector": None,
            "optionable": True,
            "score": 80.0,
            "classification": "high_conviction",
            "direction": "long",
            "horizon_min": 240,
            "snapshot": {},
            "degradations_active": [],
            "source": "quant",
            "trace_id": "trace-f13",
            "timestamp": "2026-04-27T13:00:00+00:00",
        }
    )
    intents = build_strategies_for_opportunity(opp)
    assert intents, "fixture: factory debe producir al menos un intent"
    return intents[0]


# ---------------------------------------------------------------------------
# Sección 1 — Config
# ---------------------------------------------------------------------------


class TestConfig:
    def test_defaults_off(self, monkeypatch):
        for k in (
            "ATLAS_LEAN_ENABLED",
            "ATLAS_LEAN_MODE",
            "ATLAS_LEAN_BIN",
            "ATLAS_LEAN_RESULTS_DIR",
            "ATLAS_LEAN_TIMEOUT_SEC",
        ):
            monkeypatch.delenv(k, raising=False)
        cfg = load_config_from_env()
        assert cfg.enabled is False
        assert cfg.mode == "mock"
        assert cfg.is_external is False
        assert cfg.is_mock is True
        assert cfg.timeout_sec == 60

    def test_enabled_with_external_bin(self, monkeypatch):
        monkeypatch.setenv("ATLAS_LEAN_ENABLED", "true")
        monkeypatch.setenv("ATLAS_LEAN_BIN", "/usr/local/bin/lean-fake")
        monkeypatch.setenv("ATLAS_LEAN_RESULTS_DIR", "/tmp/lean-results")
        monkeypatch.delenv("ATLAS_LEAN_MODE", raising=False)
        cfg = load_config_from_env()
        assert cfg.enabled is True
        assert cfg.mode == "external"
        assert cfg.is_external is True

    def test_enabled_without_bin_falls_to_mock(self, monkeypatch):
        monkeypatch.setenv("ATLAS_LEAN_ENABLED", "true")
        monkeypatch.delenv("ATLAS_LEAN_BIN", raising=False)
        monkeypatch.delenv("ATLAS_LEAN_MODE", raising=False)
        cfg = load_config_from_env()
        assert cfg.enabled is True
        assert cfg.mode == "mock"
        assert cfg.is_external is False

    def test_invalid_timeout_falls_back(self, monkeypatch):
        monkeypatch.setenv("ATLAS_LEAN_TIMEOUT_SEC", "not-a-number")
        cfg = load_config_from_env()
        assert cfg.timeout_sec == 60


# ---------------------------------------------------------------------------
# Sección 2 — Parser
# ---------------------------------------------------------------------------


class TestParser:
    def test_statistics_lean_format(self):
        payload = {
            "Statistics": {
                "Sharpe Ratio": "1.85",
                "Win Rate": "62.5%",
                "Drawdown": "-12.30%",
                "Net Profit": "23.45%",
                "Expectancy": "0.31",
            }
        }
        metrics, warnings = parse_statistics_payload(payload)
        assert metrics["sharpe"] == pytest.approx(1.85)
        assert metrics["win_rate"] == pytest.approx(62.5)
        assert metrics["max_drawdown"] == pytest.approx(-12.30)
        assert metrics["total_return"] == pytest.approx(23.45)
        assert metrics["expectancy"] == pytest.approx(0.31)
        assert warnings == []

    def test_statistics_flat_keys(self):
        metrics, _ = parse_statistics_payload(
            {
                "sharpe": 0.4,
                "win_rate": 50.0,
                "max_drawdown": -10.0,
                "total_return": 5.0,
                "expectancy": 0.1,
            }
        )
        assert metrics["sharpe"] == 0.4

    def test_statistics_empty(self):
        metrics, warnings = parse_statistics_payload(None)
        assert metrics == {
            "sharpe": 0.0,
            "win_rate": 0.0,
            "max_drawdown": 0.0,
            "total_return": 0.0,
            "expectancy": 0.0,
        }
        assert "statistics_payload_empty_or_invalid" in warnings

    def test_statistics_string_json(self):
        metrics, warnings = parse_statistics_payload(
            json.dumps({"Statistics": {"Sharpe Ratio": "0.9"}})
        )
        assert metrics["sharpe"] == pytest.approx(0.9)
        assert warnings == []

    def test_orders_dict_with_orders_key(self):
        orders, warnings = parse_orders_payload(
            {"orders": [{"id": 1}, {"id": 2}]}
        )
        assert len(orders) == 2
        assert warnings == []

    def test_orders_list(self):
        orders, _ = parse_orders_payload([{"id": 7}])
        assert orders == [{"id": 7}]

    def test_orders_invalid(self):
        orders, warnings = parse_orders_payload(12345)
        assert orders == []
        assert "orders_payload_not_list" in warnings

    def test_orders_invalid_json(self):
        orders, warnings = parse_orders_payload("not-json")
        assert orders == []
        assert "orders_payload_invalid_json" in warnings

    def test_parse_run_artifacts_combines(self):
        artifacts = parse_run_artifacts(
            statistics={"Statistics": {"Sharpe Ratio": "2.1"}},
            orders=[{"id": "a"}, {"id": "b"}, {"id": "c"}],
        )
        assert isinstance(artifacts, LeanRunArtifacts)
        assert artifacts.sharpe == pytest.approx(2.1)
        assert artifacts.num_orders == 3


# ---------------------------------------------------------------------------
# Sección 3 — run_backtest_for_strategy_intent
# ---------------------------------------------------------------------------


class TestRunBacktest:
    def test_disabled_when_flag_off(self, monkeypatch):
        monkeypatch.delenv("ATLAS_LEAN_ENABLED", raising=False)
        intent = _make_intent()
        result = run_backtest_for_strategy_intent(intent)
        assert result.success is False
        assert result.mode == "disabled"
        assert result.error_code == ERROR_DISABLED

    def test_invalid_intent_none(self):
        cfg = LeanAdapterConfig(enabled=True, mode="mock")
        result = run_backtest_for_strategy_intent(None, config=cfg)
        assert result.success is False
        assert result.error_code == ERROR_INVALID_INTENT

    def test_invalid_intent_garbage(self):
        cfg = LeanAdapterConfig(enabled=True, mode="mock")
        result = run_backtest_for_strategy_intent("not-an-intent", config=cfg)  # type: ignore[arg-type]
        assert result.success is False
        assert result.error_code == ERROR_INVALID_INTENT

    def test_mock_deterministic(self):
        cfg = LeanAdapterConfig(enabled=True, mode="mock")
        intent = _make_intent()
        r1 = run_backtest_for_strategy_intent(intent, config=cfg)
        r2 = run_backtest_for_strategy_intent(intent, config=cfg)
        assert r1.success is True
        assert r1.mode == "mock"
        assert r1.sharpe == r2.sharpe
        assert r1.win_rate == r2.win_rate
        assert r1.max_drawdown == r2.max_drawdown
        assert r1.total_return == r2.total_return
        # rangos plausibles
        assert -1.0 <= r1.sharpe <= 3.0
        assert 0.0 <= r1.win_rate <= 100.0
        assert -30.0 <= r1.max_drawdown <= 0.0

    def test_external_happy_path(self, tmp_path, monkeypatch):
        # Preparamos artefactos en results_dir
        (tmp_path / "statistics.json").write_text(
            json.dumps(
                {
                    "Statistics": {
                        "Sharpe Ratio": "1.10",
                        "Win Rate": "55%",
                        "Drawdown": "-7%",
                        "Net Profit": "12%",
                        "Expectancy": "0.22",
                    }
                }
            ),
            "utf-8",
        )
        (tmp_path / "orders.json").write_text(
            json.dumps([{"id": 1}, {"id": 2}]), "utf-8"
        )

        class FakeProc:
            returncode = 0
            stderr = ""
            stdout = ""

        with patch("subprocess.run", return_value=FakeProc()) as run_mock:
            cfg = LeanAdapterConfig(
                enabled=True,
                mode="external",
                bin_path="/fake/lean",
                results_dir=str(tmp_path),
                timeout_sec=5,
            )
            intent = _make_intent()
            result = run_backtest_for_strategy_intent(intent, config=cfg)

        assert run_mock.called
        assert result.success is True
        assert result.mode == "external"
        assert result.sharpe == pytest.approx(1.10)
        assert result.num_orders == 2

    def test_external_timeout(self, monkeypatch, tmp_path):
        cfg = LeanAdapterConfig(
            enabled=True,
            mode="external",
            bin_path="/fake/lean",
            results_dir=str(tmp_path),
            timeout_sec=1,
        )

        def boom(*a, **kw):
            raise subprocess.TimeoutExpired(cmd=["lean"], timeout=1)

        with patch("subprocess.run", side_effect=boom):
            result = run_backtest_for_strategy_intent(_make_intent(), config=cfg)
        assert result.success is False
        assert result.error_code == ERROR_TIMEOUT

    def test_external_non_zero_exit(self, tmp_path):
        cfg = LeanAdapterConfig(
            enabled=True,
            mode="external",
            bin_path="/fake/lean",
            results_dir=str(tmp_path),
        )

        class BadProc:
            returncode = 2
            stderr = "fail"
            stdout = ""

        with patch("subprocess.run", return_value=BadProc()):
            result = run_backtest_for_strategy_intent(_make_intent(), config=cfg)
        assert result.success is False
        assert result.error_code == ERROR_PROCESS_FAILED

    def test_external_binary_missing(self, tmp_path):
        cfg = LeanAdapterConfig(
            enabled=True,
            mode="external",
            bin_path="/no/such/binary",
            results_dir=str(tmp_path),
        )
        with patch(
            "subprocess.run", side_effect=FileNotFoundError("missing")
        ):
            result = run_backtest_for_strategy_intent(_make_intent(), config=cfg)
        assert result.success is False
        assert result.error_code == ERROR_PROCESS_FAILED

    def test_external_without_bin_path(self, tmp_path):
        # mode external pero bin_path None → fallo honesto, sin subprocess
        cfg = LeanAdapterConfig(
            enabled=True,
            mode="external",
            bin_path=None,
            results_dir=str(tmp_path),
        )
        with patch("subprocess.run") as run_mock:
            result = run_backtest_for_strategy_intent(_make_intent(), config=cfg)
        assert run_mock.called is False
        assert result.success is False


# ---------------------------------------------------------------------------
# Sección 4 — Aislamiento
# ---------------------------------------------------------------------------


_F13_MODULES = [
    Path("atlas_code_quant/lean/config.py"),
    Path("atlas_code_quant/lean/parser/results.py"),
    Path("atlas_code_quant/lean/runner/launcher.py"),
]

_PROHIBITED_IMPORTS = (
    "atlas_code_quant.execution",
    "atlas_code_quant.operations",
    "atlas_code_quant.autonomy",
    "atlas_code_quant.risk",
    "atlas_code_quant.vision",
    "atlas_adapter",
    "tradier",
    "broker_router",
    "live_loop",
    "live_activation",
)


@pytest.mark.parametrize("rel", [str(p) for p in _F13_MODULES])
def test_f13_modules_have_no_prohibited_imports(rel):
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / rel).read_text("utf-8")
    tree = ast.parse(src)
    bad: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            for prohibited in _PROHIBITED_IMPORTS:
                if mod == prohibited or mod.startswith(prohibited + "."):
                    bad.append(mod)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                for prohibited in _PROHIBITED_IMPORTS:
                    if alias.name == prohibited or alias.name.startswith(
                        prohibited + "."
                    ):
                        bad.append(alias.name)
    assert not bad, f"{rel} importa módulos prohibidos: {bad}"
