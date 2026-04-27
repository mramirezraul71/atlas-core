# ATLAS_CODE_QUANT_F15 — Tradier institutional adapter (paper / dry-run)

## Política F15
- F15 introduce un adapter Tradier **paralelo y aislado** para uso en
  papel: `atlas_code_quant/execution/tradier_adapter.py`.
- **No reemplaza ni toca** el stack F4 canónico
  (`tradier_execution`, `tradier_controls`, `tradier_pdt_ledger`,
  `broker_router`, `signal_executor`, `operation_center`,
  `auton_executor`, `live_activation`, `live_loop`,
  `start_paper_trading`). El adapter F15 es independiente.
- Default `dry_run=True`. Salir de dry-run requiere
  `ATLAS_TRADIER_DRYRUN=false` explícitamente — F15 NO ejecuta
  órdenes reales en runtime.
- F15 NO toca locks (`paper_only`, `full_live_globally_locked`).

## Alcance
Toca SÓLO:
- `atlas_code_quant/execution/tradier_adapter.py` (nuevo)
- `tests/atlas_code_quant/test_tradier_adapter_f15.py` (nuevo, 18 tests)
- `docs/ATLAS_CODE_QUANT_F15_TRADIER_ADAPTER_INSTITUTIONAL.md` (este doc)

## API pública

```python
from atlas_code_quant.execution.tradier_adapter import (
    TradierAdapter,
    TradierAdapterConfig,
    OrderIntent,
    OrderResult,
    load_config_from_env,
)
```

### `TradierAdapterConfig`

| campo | default | env var |
|---|---|---|
| `base_url` | `https://sandbox.tradier.com` | `ATLAS_TRADIER_BASE_URL` |
| `token` | `""` | `ATLAS_TRADIER_TOKEN` |
| `account_id` | `""` | `ATLAS_TRADIER_ACCOUNT_ID` |
| `dry_run` | `True` | `ATLAS_TRADIER_DRYRUN` |
| `max_orders_per_minute` | `30` | `ATLAS_TRADIER_MAX_OPM` |
| `max_retries_5xx` | `3` | — |
| `retry_base_sleep_sec` | `0.05` | — |
| `retry_max_sleep_sec` | `1.0` | — |
| `request_timeout_sec` | `5.0` | — |

### `OrderIntent`
- Campos: `symbol`, `side`, `quantity`, `order_type`, `duration`,
  `trace_id`, `asset_class`, `legs`, `metadata`.
- `is_valid()` valida símbolo no vacío, qty > 0 y `side` ∈
  {`buy`, `sell`, `buy_to_open`, `sell_to_open`, `buy_to_close`,
  `sell_to_close`}.
- `idempotency_key()` = SHA256 truncado de
  `(trace_id, symbol, side, quantity, order_type, asset_class)` →
  estable bajo reintentos.

### `TradierAdapter.submit(intent) -> OrderResult`

| escenario | comportamiento |
|---|---|
| `intent` inválido | `ok=False`, `error_code=TRADIER_INVALID_INTENT`, sin HTTP |
| `dry_run=True` | `ok=True`, `dry_run=True`, `order_id=DRY-<idem[:12]>`, `error_code=TRADIER_DRY_RUN_OK`, sin HTTP |
| Rate limit lógico saturado | `ok=False`, `error_code=TRADIER_RATE_LIMITED`, sin HTTP |
| HTTP 2xx | `ok=True`, `order_id` desde `payload["order"]["id"]`, `attempts=N` |
| HTTP 4xx | `ok=False`, `error_code=TRADIER_BAD_REQUEST`, **sin retry** |
| HTTP 5xx | retries con backoff exponencial + jitter; agotados → `error_code=TRADIER_HTTP_5XX` |
| Transport error | `ok=False`, `error_code=TRADIER_TRANSPORT_ERROR`, sin retry agresivo |

Headers enviados: `Authorization: Bearer <token>`, `Accept`,
**`X-Idempotency-Key`**.

### `TradierAdapter.reconcile_positions() -> dict`
- En dry-run: devuelve estado interno mínimo (positions vacío hasta
  que se conecte la fase de journal). No toca Tradier real.
- En live: GET `/v1/accounts/{id}/positions` y persiste el snapshot
  en estado interno. F15 no integra esto en ningún scheduler.

## Tests F15 — 18 casos

- Config: defaults dry-run; `ATLAS_TRADIER_DRYRUN=false` → live.
- OrderIntent: válido, símbolo vacío, qty 0, side inválido,
  idempotency estable bajo mismo input y diferente bajo qty distinta.
- Adapter dry-run: respuesta sintética con prefijo `DRY-`,
  propaga `idempotency_key`; intent inválido → error.
- Adapter live (httpx.MockTransport):
  - 200 happy path → ok, order_id propagado, headers contienen
    `X-Idempotency-Key`.
  - 400 → `TRADIER_BAD_REQUEST` sin retry.
  - 503 persistente con `max_retries_5xx=2` → 1 + 2 = 3 calls,
    `TRADIER_HTTP_5XX`.
  - 503 → 200 → ok=True con `attempts=2`.
  - `httpx.ConnectError` → `TRADIER_TRANSPORT_ERROR`.
  - rate limit lógico (`max_opm=2`, 3 intents) → 2 ok, 3º
    `TRADIER_RATE_LIMITED`, sin tocar HTTP.
- Reconcile dry-run.
- AST guard: F15 NO importa `tradier_execution`, `tradier_controls`,
  `tradier_pdt_ledger`, `broker_router`, `signal_executor`,
  `operation_center`, `auton_executor`, `live_activation`,
  `live_loop`, `start_paper_trading`, `atlas_adapter`.
- AST guard: identificadores `paper_only` /
  `full_live_globally_locked` NO aparecen como Name/Attribute en F15.

## Invariantes preservadas
- Stack F4 canónico Tradier intacto: F15 NO modifica
  `tradier_execution.py` ni los módulos asociados.
- F3..F14 intactos.
- Sleep en backoff inyectable (por defecto `time.sleep`); en tests
  se reemplaza con no-op.
- HTTPX usado vía `httpx.MockTransport` para tests deterministas.

## Rollback

```bash
git -C /home/user/workspace/atlas-core revert <COMMIT_F15>
```

Sin impacto sobre runtime: F15 no es invocado por
`broker_router`, `signal_executor`, `operation_center`,
`auton_executor`, `live_activation`, ni el resto del stack F4.

## Próximos pasos
- **F16**: paper execution pipeline end-to-end:
  Radar (gated F10) → factory (F12) → fitness (F14) →
  `TradierAdapter.submit` (F15, dry-run).
