"""Tests F15 — Tradier institutional adapter (paper / dry-run).

Cubre:

* Config: defaults (dry_run=True), parsing de env vars, ``is_live``.
* OrderIntent: validación, idempotency_key estable.
* TradierAdapter dry-run: respuesta sintética con ``DRY-`` prefix,
  no llama HTTP, propaga idempotency_key.
* Adapter live (mock httpx via MockTransport):
    - happy path 200 → ok=True, order_id propagado.
    - bad request 400 → ok=False, ERROR_BAD_REQUEST, sin retry.
    - 5xx → retry hasta agotar y devuelve ERROR_HTTP_5XX.
    - 5xx-luego-200 → ok=True con attempts>1.
    - transport error → ERROR_TRANSPORT.
    - rate limit lógico → ERROR_RATE_LIMITED sin tocar HTTP.
    - X-Idempotency-Key presente en headers.
* reconcile_positions dry-run.
* AST guard: NO importar broker_router / tradier_execution canónico /
  signal_executor / operation_center / live_*.
"""

from __future__ import annotations

import ast
from pathlib import Path

import httpx
import pytest

from atlas_code_quant.execution.tradier_adapter import (
    ERROR_BAD_REQUEST,
    ERROR_DRY_RUN_OK,
    ERROR_HTTP_5XX,
    ERROR_INVALID_INTENT,
    ERROR_RATE_LIMITED,
    ERROR_TRANSPORT,
    OrderIntent,
    OrderResult,
    TradierAdapter,
    TradierAdapterConfig,
    load_config_from_env,
)


# ---------------------------------------------------------------------------
# Sección 1 — Config
# ---------------------------------------------------------------------------


class TestConfig:
    def test_defaults_dry_run(self, monkeypatch):
        for k in (
            "ATLAS_TRADIER_BASE_URL",
            "ATLAS_TRADIER_TOKEN",
            "ATLAS_TRADIER_ACCOUNT_ID",
            "ATLAS_TRADIER_DRYRUN",
            "ATLAS_TRADIER_MAX_OPM",
        ):
            monkeypatch.delenv(k, raising=False)
        cfg = load_config_from_env()
        assert cfg.dry_run is True
        assert cfg.is_live is False
        assert cfg.max_orders_per_minute == 30
        assert cfg.base_url.endswith("tradier.com")

    def test_explicit_disable_dry_run(self, monkeypatch):
        monkeypatch.setenv("ATLAS_TRADIER_DRYRUN", "false")
        cfg = load_config_from_env()
        assert cfg.dry_run is False
        assert cfg.is_live is True


# ---------------------------------------------------------------------------
# Sección 2 — OrderIntent
# ---------------------------------------------------------------------------


class TestOrderIntent:
    def test_valid(self):
        intent = OrderIntent(symbol="SPY", side="buy", quantity=1)
        assert intent.is_valid() is True

    def test_invalid_symbol(self):
        assert OrderIntent(symbol="", side="buy", quantity=1).is_valid() is False

    def test_invalid_quantity(self):
        assert (
            OrderIntent(symbol="SPY", side="buy", quantity=0).is_valid() is False
        )

    def test_invalid_side(self):
        assert (
            OrderIntent(symbol="SPY", side="open", quantity=1).is_valid() is False
        )

    def test_idempotency_stable(self):
        a = OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id="t1")
        b = OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id="t1")
        c = OrderIntent(symbol="SPY", side="buy", quantity=2, trace_id="t1")
        assert a.idempotency_key() == b.idempotency_key()
        assert a.idempotency_key() != c.idempotency_key()


# ---------------------------------------------------------------------------
# Sección 3 — Adapter dry-run
# ---------------------------------------------------------------------------


class TestAdapterDryRun:
    def test_dry_run_no_http(self):
        # Sin client, dry_run=True: nunca debe llegar a HTTP.
        cfg = TradierAdapterConfig(dry_run=True, max_orders_per_minute=10)
        adapter = TradierAdapter(cfg)
        intent = OrderIntent(
            symbol="SPY", side="buy", quantity=1, trace_id="trace-a"
        )
        out = adapter.submit(intent)
        assert isinstance(out, OrderResult)
        assert out.ok is True
        assert out.dry_run is True
        assert out.error_code == ERROR_DRY_RUN_OK
        assert out.order_id is not None and out.order_id.startswith("DRY-")
        assert out.idempotency_key == intent.idempotency_key()

    def test_invalid_intent(self):
        adapter = TradierAdapter(TradierAdapterConfig(dry_run=True))
        out = adapter.submit("oops")  # type: ignore[arg-type]
        assert out.ok is False
        assert out.error_code == ERROR_INVALID_INTENT


# ---------------------------------------------------------------------------
# Sección 4 — Adapter live (mock httpx)
# ---------------------------------------------------------------------------


def _make_adapter_with_transport(
    handler,
    *,
    dry_run: bool = False,
    max_opm: int = 30,
    max_retries: int = 3,
):
    transport = httpx.MockTransport(handler)
    client = httpx.Client(transport=transport, base_url="https://sandbox.tradier.com")
    cfg = TradierAdapterConfig(
        base_url="https://sandbox.tradier.com",
        token="t",
        account_id="ACC",
        dry_run=dry_run,
        max_orders_per_minute=max_opm,
        max_retries_5xx=max_retries,
        retry_base_sleep_sec=0.0,
        retry_max_sleep_sec=0.0,
    )
    # Sleep no-op para tests.
    return TradierAdapter(cfg, client=client, sleep_fn=lambda *_a, **_kw: None)


class TestAdapterLive:
    def test_happy_path(self):
        seen_headers: dict[str, str] = {}

        def handler(request: httpx.Request) -> httpx.Response:
            seen_headers.update(request.headers)
            return httpx.Response(
                200, json={"order": {"id": "12345", "status": "ok"}}
            )

        adapter = _make_adapter_with_transport(handler)
        intent = OrderIntent(
            symbol="SPY", side="buy", quantity=1, trace_id="trace-h"
        )
        out = adapter.submit(intent)
        assert out.ok is True
        assert out.dry_run is False
        assert out.http_status == 200
        assert out.order_id == "12345"
        assert out.attempts == 1
        # Idempotency en header.
        assert (
            seen_headers.get("x-idempotency-key") == intent.idempotency_key()
            or seen_headers.get("X-Idempotency-Key")
            == intent.idempotency_key()
        )

    def test_bad_request_no_retry(self):
        calls = {"n": 0}

        def handler(request: httpx.Request) -> httpx.Response:
            calls["n"] += 1
            return httpx.Response(400, text="bad")

        adapter = _make_adapter_with_transport(handler)
        out = adapter.submit(
            OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id="t")
        )
        assert out.ok is False
        assert out.error_code == ERROR_BAD_REQUEST
        assert out.http_status == 400
        assert calls["n"] == 1

    def test_5xx_exhausts_retries(self):
        calls = {"n": 0}

        def handler(request: httpx.Request) -> httpx.Response:
            calls["n"] += 1
            return httpx.Response(503, text="busy")

        adapter = _make_adapter_with_transport(handler, max_retries=2)
        out = adapter.submit(
            OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id="t")
        )
        assert out.ok is False
        assert out.error_code == ERROR_HTTP_5XX
        assert out.http_status == 503
        # max_retries=2 → 1 inicial + 2 reintentos = 3 calls
        assert calls["n"] == 3
        assert out.attempts == 3

    def test_5xx_then_200(self):
        seq = iter([503, 200])

        def handler(request: httpx.Request) -> httpx.Response:
            code = next(seq)
            if code == 200:
                return httpx.Response(200, json={"order": {"id": "ok-1"}})
            return httpx.Response(503, text="fail-once")

        adapter = _make_adapter_with_transport(handler, max_retries=3)
        out = adapter.submit(
            OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id="t")
        )
        assert out.ok is True
        assert out.attempts == 2
        assert out.order_id == "ok-1"

    def test_transport_error(self):
        def handler(request: httpx.Request) -> httpx.Response:
            raise httpx.ConnectError("boom", request=request)

        adapter = _make_adapter_with_transport(handler)
        out = adapter.submit(
            OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id="t")
        )
        assert out.ok is False
        assert out.error_code == ERROR_TRANSPORT

    def test_rate_limited_no_http(self):
        calls = {"n": 0}

        def handler(request: httpx.Request) -> httpx.Response:
            calls["n"] += 1
            return httpx.Response(200, json={"order": {"id": "x"}})

        adapter = _make_adapter_with_transport(handler, max_opm=2)
        intents = [
            OrderIntent(symbol="SPY", side="buy", quantity=1, trace_id=f"t{i}")
            for i in range(3)
        ]
        results = [adapter.submit(it) for it in intents]
        # 2 primeros pasan, el 3º cae por rate limit sin ir a HTTP.
        ok_count = sum(1 for r in results if r.ok)
        assert ok_count == 2
        assert results[-1].error_code == ERROR_RATE_LIMITED
        assert calls["n"] == 2


# ---------------------------------------------------------------------------
# Sección 5 — reconcile_positions
# ---------------------------------------------------------------------------


class TestReconcile:
    def test_dry_run_reconcile(self):
        adapter = TradierAdapter(TradierAdapterConfig(dry_run=True))
        out = adapter.reconcile_positions()
        assert out["ok"] is True
        assert out["dry_run"] is True
        assert "positions" in out


# ---------------------------------------------------------------------------
# Sección 6 — Aislamiento AST
# ---------------------------------------------------------------------------


_F15_MODULE = Path("atlas_code_quant/execution/tradier_adapter.py")


_PROHIBITED_IMPORTS = (
    "atlas_code_quant.execution.tradier_execution",
    "atlas_code_quant.execution.tradier_controls",
    "atlas_code_quant.execution.tradier_pdt_ledger",
    "atlas_code_quant.execution.broker_router",
    "atlas_code_quant.execution.signal_executor",
    "atlas_code_quant.operations.operation_center",
    "atlas_code_quant.operations.auton_executor",
    "atlas_code_quant.production.live_activation",
    "atlas_code_quant.execution.live_loop",
    "atlas_code_quant.start_paper_trading",
    "atlas_adapter",
)


def test_tradier_adapter_isolation():
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F15_MODULE).read_text("utf-8")
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
    assert not bad, f"tradier_adapter importa prohibidos: {bad}"


def test_tradier_adapter_does_not_use_locks_as_code():
    """Los locks NO deben aparecer como Name/Attribute en F15."""
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F15_MODULE).read_text("utf-8")
    tree = ast.parse(src)
    forbidden = {"paper_only", "full_live_globally_locked"}
    bad: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id in forbidden:
            bad.append(node.id)
        if isinstance(node, ast.Attribute) and node.attr in forbidden:
            bad.append(node.attr)
    assert not bad, f"tradier_adapter referencia locks prohibidos: {bad}"
