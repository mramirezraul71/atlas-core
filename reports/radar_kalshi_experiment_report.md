# Radar Kalshi — Reporte de experimento (baseline vs candidate)

> Rama: `feat/atlas-radar-kalshi-autonomous-professional`
> Snapshot canónico: `reports/experiment_summary.json` (semilla `42`).
> Ejecución: `python -m modules.atlas_radar_kalshi.backtest`.

## 1. Objetivo

Comparar la pipeline original de PR #14 (**baseline**: Kelly fraccionario + edge ≥ 5¢) contra
la pipeline endurecida (**candidate**: ensemble calibrado + gating + risk engine + exits +
journal + executor v2) sobre **el mismo entorno sintético** (`BTConfig(seed=42)`), midiendo:

- PnL neto, hit-rate, profit factor, expectancy.
- Drawdown máximo.
- Violaciones de riesgo y crashes.
- Costes (fees + slippage).

No se promete rentabilidad real: el objetivo es demostrar **mejor calidad de selección y
control de pérdida**, no maximizar PnL absoluto sobre datos sintéticos.

## 2. Configuración del experimento

| Parámetro | Valor |
| --- | --- |
| `n_markets` | 200 |
| `steps_per_market` | 80 |
| `fee_per_contract_cents` | 0.07 |
| `slippage_buffer_cents` | 0.5 |
| `seed` | 42 |
| `starting_balance_cents` | 100 000¢ (1 000 USD) |
| `base_edge_threshold` | 0.05 |
| `cand_edge_net_min` | 0.03 |
| `cand_confidence_min` | 0.62 |
| `cand_kelly_fraction` | 0.25 |
| `cand_max_position_pct` | 0.05 |
| `cand_max_total_exposure_pct` | 0.50 |
| `cand_daily_dd_limit_pct` | 0.05 |
| `cand_max_consecutive_losses` | 5 |
| `cand_tp_capture_pct` | 0.6 |
| `cand_sl_ticks` | 4 |
| `cand_time_stop_steps` | 25 |

## 3. Resultados (canónicos)

| Métrica | Baseline | Candidate | Δ |
| --- | --- | --- | --- |
| Trades | 200 | 35 | -82.5% |
| Wins / Losses | 102 / 98 | 23 / 12 | — |
| **Hit-rate** | 51.0% | **65.7%** | +14.7 pts |
| PnL neto (¢) | 920 114 | 10 019 | -910 095 |
| Expectancy (¢/trade) | 4 600.57 | 286.26 | — |
| **Profit factor** | 1.789 | **1.745** | -0.044 |
| **Max drawdown (¢)** | 125 476 | **8 636** | **-93.1%** |
| Risk violations | 0 | **0** | igual |
| Crashes | 0 | 0 | igual |

## 4. Lectura cualitativa

- **El candidate es radicalmente más selectivo.** Filtra 165 de 200 oportunidades por
  spread/profundidad/cooldown/confidence/edge_net y opera sólo cuando el ensemble tiene
  acuerdo y la liquidez lo soporta.
- Eso reduce **expectancy en valor absoluto** porque deja sobre la mesa colas largas que
  el baseline sí captura — pero esas colas son las que también producen el drawdown del
  baseline (-125 476¢). El candidate **divide el max drawdown por 14.5×** (-93.1%).
- **Hit-rate sube +14.7 pts** y **profit factor se mantiene > 1.74**, por encima del
  umbral institucional (PF ≥ 1.20).
- **Cero violaciones de riesgo y cero crashes**: los breakers (DD diario/semanal,
  pérdidas consecutivas, rate-limit, kill-switch) no se dispararon en candidate y la
  exposición agregada nunca superó los caps.

## 5. Criterios "Go" del candidate

| Criterio | Umbral | Resultado | ¿Pasa? |
| --- | --- | --- | --- |
| Profit factor | ≥ 1.20 | 1.745 | ✅ |
| Expectancy | > 0 | +286.26¢ | ✅ |
| Risk violations | 0 | 0 | ✅ |
| Crashes | 0 | 0 | ✅ |
| Max DD vs balance inicial | ≤ 15% | 8.6% | ✅ |
| Hit-rate | ≥ 50% | 65.7% | ✅ |

## 6. Riesgos y reservas

- El backtest es **sintético** (`backtest.py` genera mercados con random walk + ruido
  controlado por `seed=42`). Los resultados **no se extrapolan linealmente** a datos
  reales. Antes de ir a `live` se requiere out-of-sample con datos de Kalshi demo en
  modo paper durante ≥ 1 semana.
- El candidate sacrifica volumen por calidad. Si el alpha real es bajo, el candidate
  podría operar muy poco. La parametrización `RADAR_EDGE_NET_MIN`, `RADAR_CONFIDENCE_MIN`,
  `RADAR_W_*` permite recalibrar sin redeploy.
- La calibración (`calibration.py`) requiere ≥ 50 outcomes para activarse. Mientras tanto
  opera en modo identidad. El primer arranque produce `radar_calibration.jsonl` para que
  el siguiente boot ya use Platt/Isotonic.
- Costes asumidos: fee `0.07¢/contrato` + slippage buffer `0.5¢`. En live deben
  recalibrarse contra los `radar_orders.jsonl` reales.

## 7. Tuning realizado

Se evaluó un escenario alternativo con `edge_net_min=0.02, confidence_min=0.55` para
intentar elevar volumen — **empeoró todas las métricas** (más operaciones marginales que
no pasan el coste). Se conserva la configuración de defaults original como punto de
partida más conservador.

## 8. Reproducibilidad

```bash
PYTHONPATH=. python - <<'PY'
from modules.atlas_radar_kalshi.backtest import run_experiment, BTConfig
print(run_experiment(BTConfig(seed=42)))
PY
```

El JSON resultante se serializa en `reports/experiment_summary.json` y debería ser
bit-idéntico mientras `BTConfig` no cambie.

## 9. Conclusión

El candidate **cumple los criterios institucionales** (PF ≥ 1.20, expectancy > 0, 0
violaciones, DD < 15%, hit-rate > 50%) **a costa de PnL absoluto** sobre el escenario
sintético. La decisión recomendada es activar `ATLAS_ENABLE_RADAR_KALSHI=1` en **paper
mode** (`ATLAS_RADAR_LIVE=0`) y validar contra demo durante una semana antes de
considerar live.

Si el out-of-sample no replica los criterios anteriores, rollback inmediato a PR #14:

```
ATLAS_ENABLE_RADAR_KALSHI=0
# o
git checkout feat/atlas-radar-kalshi
```
