# ATLAS_CODE_QUANT_F11 — StrategyIntent sobre RadarOpportunityInternal

## Política F11
- Scanner propone, **Radar decide** (ya enforced en F10 a nivel API).
- F11 introduce la **interfaz canónica** de estrategia que toma como
  input una `RadarOpportunityInternal` ya aprobada por Radar.
- F11 **no** ejecuta nada: no llama LEAN (eso es F13), no llama
  Tradier (eso es F15), no consulta precios reales y no toca
  `execution/`, `operations/`, `autonomy/`, `risk/`, `vision/`,
  `atlas_adapter/`, `broker_router/`, `tradier_*`, `live_loop`,
  `live_activation` ni los locks (`paper_only`,
  `full_live_globally_locked`).

## Alcance
Toca SÓLO:
- `atlas_code_quant/strategies/options/intent.py` (nuevo)
- `atlas_code_quant/strategies/options/builder.py` (nuevo)
- `tests/atlas_code_quant/test_strategy_intent_f11.py` (nuevo)
- `docs/ATLAS_CODE_QUANT_F11_STRATEGY_INTENT_OVER_RADAR.md` (este doc)

## Modelos canónicos

### `OpportunityRef`
Referencia mínima a la oportunidad Radar de origen. No clona el
`raw` payload del Radar. Campos: `symbol`, `asset_class`, `direction`,
`horizon_min`, `classification`, `score`, `trace_id`.

### `StrategyLeg`
Pata declarativa con strikes y expiries **relativos**:
- `side`: `"buy"` | `"sell"`
- `right`: `"call"` | `"put"`
- `strike_offset_steps`: entero alrededor del ATM (positivo = OTM call /
  ITM put)
- `expiry_rel_dte`: días al vencimiento (no fecha calendario)
- `quantity`: por defecto 1
- `notes`: anotación libre (ej. `"long_atm"`, `"short_otm_+2"`)

### `RiskLimits`
- `max_loss_usd`: F11 deja `None` (no resuelve dollar risk).
- `target_r_multiple`: hint para fitness en F14.
- `sizing_hint`: hint declarativo (`"defined_risk_long"`,
  `"defined_risk_neutral"`, `"vol_long"`, `"single_long"`,
  `"single_contract"`).

### `StrategyIntent`
- `opportunity: OpportunityRef`
- `strategy_type: StrategyType` — `vertical_spread`, `iron_condor`,
  `iron_butterfly`, `straddle`, `strangle`, `long_call`, `long_put`.
- `legs: tuple[StrategyLeg, ...]`
- `risk_limits: RiskLimits`
- `metadata: Mapping[str, Any]`
- Helpers: `num_legs`, `is_multi_leg`, `with_metadata(...)`,
  `to_dict()`.

## Builders puros

Todos siguen el contrato:
```
build_*_intents_from_opportunity(opp: RadarOpportunityInternal | None)
    -> list[StrategyIntent]
```
- Defensivos: input `None`, símbolo vacío, tipo erróneo → `[]`.
- Cualquier excepción interna se traduce a `[]` + `logger.warning`
  (helper `_wrap`) — NUNCA propagan.
- Strikes siempre relativos al ATM, expiries siempre relativos en DTE.

### `choose_dte_for_horizon(horizon_min)`

| `horizon_min` | DTE bucket |
|---|---|
| `None` o ≤ 0 o no parseable | `DTE_MEDIUM = 21` |
| ≤ 4h (240 min) | `DTE_SHORT = 7` |
| ≤ 5d (7200 min) | `DTE_MEDIUM = 21` |
| > 5d | `DTE_LONG = 45` |

### Reglas de dispatch por estrategia

| Estrategia | Cuándo se propone | Patas |
|---|---|---|
| `vertical_spread` | `direction ∈ {long, short}` | 2 (call-call o put-put), debit |
| `long_single` (`long_call` / `long_put`) | `direction ∈ {long, short}` | 1, ATM |
| `iron_condor` | `direction == neutral` | 4 (puts -1/-3 + calls +1/+3) |
| `iron_butterfly` | `direction == neutral` y DTE ≠ LONG | 4 (short ATM straddle + alas ±2) |
| `straddle` | `direction == neutral` | 2 ATM (call+put long) |
| `strangle` | `direction == neutral` y DTE ≠ SHORT | 2 OTM ±2 (call+put long) |

## Tests F11 — `tests/atlas_code_quant/test_strategy_intent_f11.py`

53 tests cubren:
- Modelos: `VALID_STRATEGY_TYPES`, `to_dict`, defaults, `with_metadata`,
  `OpportunityRef` desde `RadarOpportunityInternal` y desde garbage.
- `choose_dte_for_horizon`: 9 casos paramétricos + garbage.
- Builders direccionales: long → bull call debit, short → bear put
  debit, neutral → vacío, garbage → vacío.
- Builders neutrales: condor sólo en neutral; butterfly bloqueado en
  DTE_LONG y en direccional; straddle ATM en neutral; strangle
  bloqueado en DTE_SHORT y en direccional.
- Robustez: 100+ oportunidades sintéticas (4 × 3 × 7 × 3 × 3 = 756
  combinaciones) — ningún builder lanza, todos los intents tienen
  `strategy_type ∈ VALID_STRATEGY_TYPES` y al menos una leg.
- Defensa-en-profundidad: si un helper interno lanza, el wrapper
  devuelve `[]` y emite warning.
- AST guard: ningún módulo F11 importa `execution`, `operations`,
  `autonomy`, `risk`, `vision`, `atlas_adapter`, `tradier`,
  `broker_router`, `live_loop` ni `live_activation`.
- AST guard: ningún identificador `paper_only` /
  `full_live_globally_locked` se usa como Name/Attribute en F11.
- `intent.py` no importa `RadarClient`.

## Invariantes preservadas
- F3 deprecation headers: intactos (F11 no toca `api/main.py`).
- F6/F7/F8/F9/F10: intactos (F11 no toca `intake/scanner_radar_*`,
  `monitoring/*`, ni `api/*`).
- Locks globales: intactos.
- `atlas_adapter` Radar multi-símbolo: no tocado.

## Rollback

```bash
git -C /home/user/workspace/atlas-core revert <COMMIT_F11>
```

Reverter F11 elimina la interfaz `StrategyIntent` y los builders. Como
ningún módulo runtime los consume, el rollback es sin impacto sobre
`/scanner/report`, Radar ni execution.

## Próximos pasos
- **F12**: `StrategyFactory.build_strategies_for_opportunity(opp)`
  unificará los 6 builders en un único dispatcher por oportunidad.
- **F13**: LEAN adapter MVP — backtesting/fitness sobre intents
  (sin tocar runtime de execution).
- **F14**: pipeline `evaluate_strategies_for_opportunity` que combina
  factory + LEAN fitness.
- **F15**: `TradierAdapter` institucional (paper/dry-run) con
  idempotency + retries 5xx + rate limit.
- **F16**: pipeline paper end-to-end (Radar → Strategy → LEAN fitness
  → Tradier paper).
