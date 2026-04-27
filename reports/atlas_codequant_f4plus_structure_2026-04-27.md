# ATLAS Code Quant — F4+ Structure Review (2026-04-27)

**Auditor:** Computer (Claude Sonnet 4.6)
**Rama base:** `feature/atlas-codequant-f3-scanner-cutover` (HEAD `fc025f90`)
**Rama trabajo:** `feature/atlas-codequant-structure-f4-plus`
**Política:** plan-first, paper-first, sin push hasta aprobación, sin merges a main.

---

## 1. Alcance

Implementación de los esqueletos auditables de fases F4 a F8 sobre la rama F3,
respetando los principios de defaults paper-first y no activación automática
de ninguna ruta live. Cada fase añade módulo + tests + documentación en
commits atómicos.

| Fase | Dominio                                  | Commit     |
|------|------------------------------------------|------------|
| F4   | LEAN external mode + parser tolerante    | `3fd8154d` |
| F5   | Strategy Factory + opciones Pydantic     | `8f9ed77f` |
| F6   | Tradier dry-run + risk limits + breaker  | `0ab4c262` |
| F7   | Vision timing gate + endpoint `/vision/confirm` | `5bd630da` |
| F8   | FSM 14 estados + 10 gates + kill switch  | `15c241f4` |

Todos los commits incluyen el sello `Auditor: Computer (Claude Sonnet 4.6)`.

---

## 2. F4 — LEAN external mode

**Ficheros añadidos / modificados:**
- `atlas_code_quant/lean/{config,launcher,parser}.py` con `LeanConfig.from_env`,
  `LeanLauncher.plan_backtest/run_backtest` (dry-run obligatorio en este
  esqueleto) y `LeanRunResult` con campos auditables.
- `atlas_code_quant/lean/templates/{option_spread,iron_condor}.py.tpl` y
  `alpha_radar.py` con `RadarAlphaSignal` puro (testeable) + `build_radar_alpha_model`
  con import lazy.
- Parser tolerante: `parse_statistics`, `parse_orders` (acepta `[...]` o
  `{Orders:[...]}`), `parse_equity_curve` (con/sin header) y
  `fitness_from_statistics` (normaliza camelCase y Title Case).

**Tests:** 11 verdes en `atlas_code_quant/lean/tests/test_lean_parser.py`.

**Defaults paper-first:** `ATLAS_LEAN_ENABLED=false`, `LeanLauncher.run_backtest`
exige `dry_run=True` salvo override explícito.

---

## 3. F5 — Strategy Factory + opciones

**Ficheros añadidos:**
- `atlas_code_quant/strategies/contracts.py` con `StrategyOpportunityRef`,
  `StrategyConfig`, `OptionLeg` y `StrategyPlan` (Pydantic v2) — formato
  auditable que sustituye dicts crudos.
- `atlas_code_quant/strategies/options/{vertical_spread,iron_condor,iron_butterfly,
  straddle_strangle}.py` con `build_plan(opportunity, config) -> StrategyPlan`.
- `atlas_code_quant/strategies/factory.py` con `_options_registry` y APIs
  `register_option`, `create_option`, `list_options`, `list_all` separadas del
  registro legacy F1 (`ma_cross`, `rl`).
- `atlas_code_quant/strategies/selection_policy.py` con criterios delta, OI,
  volumen, spread bid-ask, IV rank y DTE.

**Reglas direccionales (build_plan):**
- `vertical_spread`: rechazado si `direction=="neutral"`.
- `iron_condor`: rechazado si `direction!="neutral"`.
- `iron_butterfly`: rechazado si direccional; expone `create_backtester()`
  como adapter al legacy `IronButterflyBacktester`.
- `straddle_strangle`: rechazado si direccional; variant configurable.

**Tests:** 14 verdes en `atlas_code_quant/strategies/tests/`.

---

## 4. F6 — Tradier adapter dry-run + risk + circuit breaker

**Ficheros añadidos en `atlas_code_quant/execution/`:**
- `tradier_adapter.py` con `TradierConfig.from_env`, `OrderRequest`, `OrderTicket`,
  `TradierAdapter.submit/cancel_all/reconcile_positions`. **No realiza HTTP**:
  - `dry_run=true` (default) → ticket `status="dry_run"`.
  - `dry_run=false` y `live_enabled=false` → `status="blocked"` con razón
    `live_disabled_by_flag`.
  - `dry_run=false` y `live_enabled=true` → `status="rejected"` con razón
    `live_http_not_implemented_in_skeleton` (la activación real entra en F6.b).
  - Rate-limit interno por ventana móvil de 60s respeta
    `ATLAS_MAX_ORDERS_PER_MINUTE` (default 30).
- `paper_broker.py` con libro de posiciones in-memory y `flatten()`.
- `reconciler.py` con `reconcile(internal, external) -> ReconcileReport`
  detectando `delta` por símbolo (drift auditable).

**Ficheros añadidos en `atlas_code_quant/risk/`:**
- `limits.py` con `RiskLimits.from_env`, `RiskState`, `TradeIntent` y
  `check(intent, state, limits) -> RiskDecision` con razones explícitas
  (`max_daily_loss_breached`, `max_open_positions_reached`,
  `position_notional_exceeds_limit`, `per_symbol_notional_exceeds_limit`).
  Las cierres (`side="close"`) siempre se permiten.
- `circuit_breaker.py` con estados `closed/open/half_open`, triggers por
  pérdidas consecutivas y fallos de orden, `cooldown_seconds`, y
  `confirm_recovery()`.

**Variables de entorno añadidas:**
`ATLAS_TRADIER_BASE_URL`, `ATLAS_TRADIER_TOKEN`, `ATLAS_TRADIER_ACCOUNT_ID`,
`ATLAS_TRADIER_DRY_RUN=true`, `ATLAS_LIVE_TRADING_ENABLED=false`,
`ATLAS_MAX_ORDERS_PER_MINUTE=30`, `ATLAS_MAX_POSITION_NOTIONAL_USD`,
`ATLAS_MAX_DAILY_LOSS_USD`, `ATLAS_MAX_OPEN_POSITIONS`,
`ATLAS_PER_SYMBOL_MAX_NOTIONAL_USD`, `ATLAS_CB_MAX_CONSEC_LOSSES`,
`ATLAS_CB_MAX_FAILED_ORDERS`, `ATLAS_CB_COOLDOWN_SECONDS`.

**Tests:** 23 verdes en `atlas_code_quant/{execution,risk}/tests/`.

---

## 5. F7 — VisionTimingGate + endpoint `/vision/confirm`

**Ficheros añadidos / modificados:**
- `atlas_code_quant/vision/timing_gate.py` reescrito con reglas:
  - `intent="exit"` → `allow` (bypass protector; `degraded:true` si cámara no OK).
  - cámara no OK + `requires_visual_confirmation=true` → `block`.
  - cámara no OK + sin requerimiento visual → `allow` con `degraded:true`.
  - cámara OK + `pattern=confirmation` → `allow` confianza alta (0.85).
  - cámara OK + `pattern=noise` → `delay`.
  - cámara OK + `setup/unknown` → `allow` confianza media (0.65).
- `atlas_code_quant/vision/imbalance.py` extendido con `ImbalanceSnapshot` y
  `classify_imbalance(score, *, buy_threshold, sell_threshold)`.
- `atlas_adapter/routes/vision_confirm.py` con `GET /vision/confirm` que recibe
  `symbol`, `intent`, `strategy`, `requires_visual_confirmation` y devuelve
  `{decision, confidence, imbalance_side, pattern, ts, degraded, camera_state, reason}`.
  Si `ATLAS_VISION_GATE_ENABLED!=true` responde paper-default `allow,degraded:true`.
- `atlas_adapter/atlas_http_api.py` registra el router.

**Tests:** 10 verdes (`vision/tests/test_timing_gate.py` y
`tests/atlas_adapter/test_vision_confirm_contract.py`).

**Default paper-first:** `ATLAS_VISION_GATE_ENABLED=false`.

---

## 6. F8 — FSM 14 estados + 10 gates + kill switch

**Ficheros modificados:**
- `atlas_code_quant/autonomy/states.py` — enum extendido a 14 estados
  (`BOOTING`, `DEGRADED`, `SCANNING`, `OPPORTUNITY_DETECTED`,
  `STRATEGY_BUILDING`, `BACKTESTING`, `PAPER_READY`, `PAPER_EXECUTING`,
  `LIVE_ARMED`, `LIVE_EXECUTING`, `MONITORING`, `EXITING`, `KILL_SWITCH`,
  `ERROR_RECOVERY`) + `ALLOWED_TRANSITIONS` adyacencia explícita.
- `atlas_code_quant/autonomy/orchestrator.py` — `transition()` validada con
  `ValueError` en transición no permitida, `transition_to()` legacy preservado,
  `trip_kill_switch()` desde cualquier estado, reset KILL_SWITCH solo si
  `allow_kill_switch_reset=True`. Trace log auditable.
- `atlas_code_quant/autonomy/gates.py` — 10 gates funcionales:
  1. `gate_kill_switch_file`
  2. `gate_broker_readiness`
  3. `gate_data_freshness`
  4. `gate_radar_score`
  5. `gate_lean_validation`
  6. `gate_risk_engine`
  7. `gate_max_daily_loss`
  8. `gate_max_position_notional`
  9. `gate_vision_timing`
  10. `gate_paper_or_live_permission`
  Encapsulados en `GateBundle` que devuelve la primera razón de bloqueo.
  Compat F1 conservada (`GateDecision`, `evaluate_live_gate`).
- `atlas_code_quant/autonomy/kill_switch.py` — `KillSwitchWatcher.from_env()`
  con TTL configurable y `force_refresh()`. Compat F1 conservada
  (`is_kill_switch_active(path)`).

**Tests:** 32 verdes en `atlas_code_quant/autonomy/tests/`.

---

## 7. Estado del suite global

Ejecutado `pytest tests/atlas_code_quant tests/atlas_adapter tests/test_atlas_f1_imports.py atlas_code_quant/`:

- **1415 passed**, 6 failed, 3 skipped.
- Los 6 fallos son **preexistentes en F3 (`fc025f90`)** y no introducidos por
  F4-F8. Listado verificado:
  - `tests/atlas_adapter/test_nexus_robot_runtime.py` (2)
  - `tests/atlas_adapter/test_trading_quant_bridge.py` (1)
  - `atlas_code_quant/tests/test_auto_cycle_startup.py` (1)
  - `atlas_code_quant/tests/test_operation_center_autonomous_guards.py` (1)
  - `atlas_code_quant/tests/test_paper_session_orchestrator.py` (1)

Los 11 + 14 + 23 + 10 + 32 = **90 tests nuevos** introducidos por F4-F8 están en verde.

---

## 8. Residual y trabajo siguiente (F4.b–F8.b)

| Fase | Pendiente futuro                                                        |
|------|-------------------------------------------------------------------------|
| F4.b | Conexión real a binario LEAN (subprocess) + ingest de outputs reales.    |
| F5.b | Selección concreta de strikes (delta picking) en `selection_policy.py`. |
| F6.b | Implementar HTTP a Tradier (orders, positions, accounts) detrás de flag.|
| F7.b | Conectar `Insta360Capture` real y rellenar `imbalance.score` desde OCR. |
| F8.b | Loop runtime que consuma `GateBundle` antes de cada transición.         |

Ninguno de los puntos `*.b` debe activarse hasta una nueva ronda plan-first
con aprobación explícita.

---

## 9. Comandos para revisar y traer la rama

```bash
# Inspección remota (lectura)
git fetch origin feature/atlas-codequant-structure-f4-plus
git log --oneline feature/atlas-codequant-f3-scanner-cutover..origin/feature/atlas-codequant-structure-f4-plus

# Ver diff completo respecto a F3
git diff feature/atlas-codequant-f3-scanner-cutover..origin/feature/atlas-codequant-structure-f4-plus

# Probar tests F4-F8 sobre un checkout local
git checkout -b atlas-codequant-structure-f4-plus origin/feature/atlas-codequant-structure-f4-plus
PYTHONPATH=. ATLAS_SKIP_LIVE_SERVICE_TESTS=1 \
  python3 -m pytest atlas_code_quant/lean/tests \
                     atlas_code_quant/strategies/tests \
                     atlas_code_quant/execution/tests \
                     atlas_code_quant/risk/tests \
                     atlas_code_quant/vision/tests \
                     atlas_code_quant/autonomy/tests \
                     tests/atlas_adapter/test_vision_confirm_contract.py -v
```

---

## 10. Política de safety reafirmada

- `ATLAS_LIVE_TRADING_ENABLED=false` en defaults (no hay live).
- `ATLAS_TRADIER_DRY_RUN=true` en defaults (no hay HTTP a broker).
- `ATLAS_RADAR_MULTI_SYMBOL_ENABLED=false`, `ATLAS_LEGACY_SCANNER_ENABLED=false`,
  `ATLAS_LEAN_ENABLED=false`, `ATLAS_VISION_GATE_ENABLED=false`.
- Push remoto pendiente de aprobación expresa del usuario tras revisión de
  diff/log; nunca a `main` ni a `variante/nueva`; sin merges, sin force-push,
  sin reescritura de historia.
