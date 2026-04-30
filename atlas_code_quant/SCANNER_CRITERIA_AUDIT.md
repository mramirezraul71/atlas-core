# ATLAS CODE QUANT — CRITERIOS DE ESCANEO (AUDITORÍA DE CÓDIGO)

**Fuente:** solo código en `atlas_code_quant/scanner/` y `config/settings.py` (referencias directas).  
**Generado:** 2026-04-15 (sesión de auditoría).

---

## 1. Ubicación y estructura

### Archivos `.py` bajo `scanner/`

| Archivo | Rol (según código) |
|---------|---------------------|
| `scanner/opportunity_scanner.py` | Servicio principal `OpportunityScannerService`: universo, prefilter, OHLCV, métodos de señal, backtest local, scoring, ciclo. |
| `scanner/universe_catalog.py` | `ScannerUniverseCatalog`: descarga NASDAQ Trader, filtra símbolos, cache JSON. |
| `scanner/asset_classifier.py` | `classify_asset()` usado al aceptar candidato y en `_check_market_hours` indirectamente no en scanner; en escáner: L1326 `classify_asset(symbol)` en candidato aceptado. |
| `scanner/etf_universe.py` | `ETF_OPTIONS_UNIVERSE` (dict) — ampliación manual de universo vía `settings`. |
| `scanner/index_universe.py` | `INDEX_PROFILES` (dict) — idem. |
| `scanner/crypto_universe.py` | `CRYPTO_SPOT_UNIVERSE` (dict) — idem. |
| `scanner/futures_universe.py` | **No referenciado** en `opportunity_scanner.py` (grep sin coincidencias). |
| `scanner/__init__.py` | Exporta `OpportunityScannerService`. |

---

## 2. Criterios de universo elegible

### 2.1 Modos de universo (`OpportunityScannerService._resolve_scan_batch`)

**Archivo:** `opportunity_scanner.py` **L623–672**

| Condición | Lógica |
|-----------|--------|
| Manual | `source != "yfinance"` **o** `universe_mode == "manual"` → símbolos = `self._config.universe` (lista desde settings). |
| Rotativo US | `source == "yfinance"` y modo no manual → catálogo `get_us_equities()`, símbolos mezclados con `_stable_symbol_mix` (**L221–225**: orden por `SHA1(symbol)`). |
| Catálogo vacío | **L639–651**: fallback a `self._config.universe`, meta `manual_fallback`. |
| Ventana rotativa | `batch_size = min(universe_batch_size, total)`; batch = slice circular con `_universe_cursor` (**L653–661**). |

### 2.2 Catálogo NASDAQ / Other (`universe_catalog.py`)

| Elemento | Línea | Criterio exacto |
|----------|-------|------------------|
| Regex símbolo | L21 | `^[A-Z][A-Z.-]{0,9}$` |
| Test issue | L134–136 | Fila con `Test Issue == "Y"` → **excluida** |
| Nombre excluido | L22–33, L40–44 | Si `security_name` contiene token en `_EXCLUDED_NAME_TOKENS` (WARRANT, RIGHT, UNIT, UNITS, PREFERRED, PREF , DEPOSITARY, DEPOSITORY, NOTE, NOTES) → **no tradeable** |
| Símbolo normalizado | L36–37 | `strip().upper().replace(".", "-")` |

**URLs:** `NASDAQ_LISTED_URL`, `OTHER_LISTED_URL` (**L18–19**).

### 2.3 Prefiltro (precio, volumen, scoring) — solo si `source == "yfinance"` y no fast-path

**Archivo:** `opportunity_scanner.py` **`_prefilter_batch` L674–767**

| Criterio | Línea | Lógica / valor |
|----------|-------|----------------|
| Fuente no yfinance | L682–688 | Toma `symbols[:prefilter_count]` sin OHLCV; meta `source_fallback`. |
| Fast-path rotativo | L694–700 | Si `universe_mode == "us_equities_rotating"` y `len(symbols) >= 40` → **sin** descarga masiva: `selected = symbols[:prefilter_count]`; meta `rotating_fastpath`. |
| Error OHLCV | L707–716 | `ohlcv_many` falla → primeros `prefilter_count`; meta `error_fallback`. |
| Mínimo barras | L721–722 | `len(df) < 70` → **skip** símbolo |
| Precio mínimo | L727–728 | `price < settings.scanner_prefilter_min_price` → **continue** |
| Volumen dólar medio | L719–731 | `min_dollar_volume = scanner_prefilter_min_dollar_volume_millions * 1_000_000`; `avg_dollar_volume = (close*volume).tail(20).mean()`; si `< min_dollar_volume` → **continue** |
| Score prefilter | L732–737 | `ret_21 = abs(pct_change(close[-1], close[-21]))`, `ret_63 = abs(pct_change(close[-1], close[-63]))`, `atr_pct = _predicted_move_pct(df, 0.6)`, `trend_bonus = 5.0` si `close[-1] > SMA50(close)[-1]` else `0`, `liquidity_bonus = min(avg_dollar_volume/1e8, 1.0)*20`, `score = ret_21*0.42 + ret_63*0.28 + atr_pct*1.7 + liquidity_bonus + trend_bonus` |
| Selección | L754–757 | Orden descendente por score; top `prefilter_count`; si vacío → `symbols[:prefilter_count]` |

### 2.4 Defaults y clamps en `config/settings.py` (TradingConfig)

| Variable | Env / default | Clamp post-init (L449–464) |
|----------|---------------|----------------------------|
| `scanner_prefilter_min_price` | `QUANT_SCANNER_PREFILTER_MIN_PRICE`, **5.0** | [0.5, 1000.0] |
| `scanner_prefilter_min_dollar_volume_millions` | `QUANT_SCANNER_PREFILTER_MIN_DOLLAR_VOLUME_M`, **10.0** | [0.1, 5000.0] |
| `scanner_universe_batch_size` | **80** | [20, 500] |
| `scanner_prefilter_count` | **20** | [8, 80] y ≤ `universe_batch_size` |
| `scanner_universe_mode` | `us_equities_rotating` | solo `manual` \| `us_equities_rotating` |
| Lista manual `scanner_universe_raw` | default **SPY,QQQ,IWM,...** L228–230 | parse CSV → upper |
| ETFs / índices / crypto extra | L237–239, L503–518 | Si flags: extiende con claves de `ETF_OPTIONS_UNIVERSE`, `INDEX_PROFILES`, `CRYPTO_SPOT_UNIVERSE` |

**No hay en scanner:** filtro explícito por market cap, OTC, penny stock distinto del precio mínimo, blacklist por nombre fuera de `universe_catalog`.

---

## 3. Contexto de mercado (dentro del escáner)

**No** hay comprobación de horario de mercado NYSE dentro de `opportunity_scanner.py` (eso está en `OperationCenter._check_market_hours`, fuera de `scanner/`).

### 3.1 Datos y marcos temporales

| Constante | Línea | Valor |
|-----------|-------|--------|
| `TIMEFRAME_ORDER` | L25 | `["1d", "4h", "1h", "15m", "5m"]` |
| `HIGHER_TIMEFRAME_MAP` | L26–32 | 5m→15m, 15m→1h, 1h→4h, 4h→1d, 1d→None |
| `HORIZON_BARS` | L33 | 5m:3, 15m:4, 1h:6, 4h:4, 1d:3 |
| Carga OHLCV | L769–792 | `limit=420` por símbolo y timeframe; `dropna` open/high/low/close |

### 3.2 Fuerza relativa (`_relative_strength_percentiles` L794–811)

- Requiere `1d` con `len(df) >= 90`.
- `fast = pct_change(close[-1], close[-21])`, `slow = pct_change(close[-1], close[-63])`.
- Score por símbolo: `fast*0.45 + slow*0.55`.
- Percentil 0–100 entre símbolos del lote actual; faltantes → **50.0**.

### 3.3 Order flow proxy (`_order_flow_snapshot` L813–937)

- Tradier quote + timesales 45 min, intervalos `1min` luego `5min`, `session_filter="all"`.
- Componentes: `spread_bps`, `quote_imbalance_pct`, `net_pressure_pct`, `price_vs_vwap_pct`.
- **Fórmula `raw_score` (L908–913):** base 50; + clip(net_pressure*0.55, -22, 22); + clip(price_vs_vwap*35, -18, 18); + clip(quote_imbalance*0.18, -10, 10); − min(spread_bps/12, 8); luego clip 0–100.
- Dirección: `score_pct >= 55` → `alcista`; `<= 45` → `bajista`; else `neutral`.

### 3.4 Volatilidad (ATR)

- `_atr(df, period=14)` **L192–205**.
- Usado en breakout Donchian, `predicted_move_pct`, prefilter.

**No hay en scanner:** VIX, SPY régimen global, earnings, dividend dates, correlación sectorial explícita.

---

## 4. Criterios de detección de setup (`_method_signal` L969–1060)

Orden de métodos: **`METHOD_ORDER` L34** = `trend_ema_stack`, `breakout_donchian`, `rsi_pullback_trend`, `ml_directional`.

| Método | Mín. datos | Condiciones (resumen literal del código) |
|--------|------------|------------------------------------------|
| `trend_ema_stack` | len≥60; EMA stack len≥220 | EMA20/50/200; `bullish`: precio > ema20 > ema50 > ema200 y slope ema20 > 0; `bearish`: análogo abajo; strength `0.58 + min(abs(slope)/2, 0.17) + min(|pct_change(ema20,ema50)|/10, 0.18)` cap 0.96 |
| `breakout_donchian` | len≥60 | High 20 períodos prev shift, low idem; long: precio > channel_high y volumen ≥ 0.9 * vol20; strength desde breakout/ATR14; short simétrico |
| `rsi_pullback_trend` | len≥60; RSI pull len≥220 | ema50/200; long: tendencia alcista y rsi2[-2]<10≤rsi2[-1]; short: tendencia baja y rsi2[-2]>90≥rsi2[-1]; strength acotado 0.88 |
| `ml_directional` | len≥60 | `MLSignalStrategy` confidence_threshold **0.55**, target_bars **5**, model **rf** (**L459–465**); dirección desde señal BUY/SELL |

Evidencia bibliográfica por método: **`EVIDENCE_LIBRARY` L35–157** (scores0.58–0.88 usados en scoring).

---

## 5. Backtest local reciente (`_backtest_method_recent` L1062–1128)

- Ventana: desde `max(warmup, len(df)-140)` hasta `len(df)-horizon` con `horizon = HORIZON_BARS[timeframe]`.
- Warmup por método: ema_stack 210, donchian 50, rsi_pullback 210, ml 80.
- Movimiento: `signed_move = ((fut/now)-1)*direction`.
- Métricas: wins, losses, win_rate, profit_factor, expectancy, payoff_ratio.
- **`quality_score_pct` L1102–1108:**  
  `0.28*win_rate + 0.24*profit_factor_score + 0.20*expectancy_score + 0.14*payoff_score + 0.14*sample_confidence`  
  con sub-scores clipados según fórmulas L1098–1101.

---

## 6. Scoring y ranking

### 6.1 Filtro previo a “mejor método” por timeframe (`_evaluate_symbol_timeframes` L1166+)

- Por cada `method` en `METHOD_ORDER`: si `direction==0` o `strength < scanner_min_signal_strength` (**L1188–1189**) → no entra en evaluaciones.
- Defaults settings (**L214–218**): `min_signal_strength` **0.55**, `min_local_win_rate_pct` **53.0**, `min_local_profit_factor` **1.05**, `min_backtest_sample` **12**, `min_selection_score` **65.0**.

### 6.2 Consenso temporal (`_timeframe_consensus` L1130–1143)

- Peso por evaluación: `strength * evidence_score`; bull vs bear; si empate o total≤0 → neutral.

### 6.3 Confirmación timeframe superior (L1223–1236)

- `alignment_score` base **55.0**; si dirección alineada con TF superior: **70 + confidence*30**; TF superior neutral: **45.0** y si `require_higher_tf_confirmation` → razón de rechazo; contradicción: **0.0** + rechazo.

### 6.4 Rechazos por estadística local (L1237–1246)

- `local_win_rate_pct < min_local_win_rate_pct` → rechazo.
- `local_profit_factor < min_local_profit_factor` → rechazo.
- `local_sample < min_backtest_sample` → rechazo.

### 6.5 Order flow vs setup (L1276–1283)

- Si `order_flow.available` y `timeframe in {5m,15m,1h}` y dirección OF alcista/bajista **≠** dirección setup y `order_flow_confidence >= 70` → rechazo.

### 6.6 `selection_score` (L1265–1275)

```
selection_score = round(
  local_quality_score * 0.44
  + relative_strength_pct * 0.14
  + strength * 100 * 0.12
  + alignment_score * 0.10
  + evidence_score * 100 * 0.06
  + order_flow_score * 0.08
  + order_flow_alignment_score * 0.06
  + adaptive_bias,
  2,
)
```

- `adaptive_bias` desde `learning.context(...)` (**L1247–1251**).

- Si `selection_score < min_selection_score` → rechazo (**L1284–1285**).

### 6.7 Umbral dinámico y top-K (`_dynamic_selection_threshold` L1150–1164; `_run_cycle` L1395–1414)

- Si hay más candidatos provisionales que `max_candidates`: sube umbral entre `base_floor`, `rank_cutoff` y `density_bonus` (hasta +10).
- Resultado final: ordenar por `selection_score` descendente, **`[:max_candidates]`** (default max_candidates **8**).

### 6.8 Movimiento previsto (`_predicted_move_pct` L1145–1148)

- `atr_pct = (ATR14/close)*100`; retorna `max(atr_pct*(0.8+strength), 0.05)`.

---

## 7. Flujo del ciclo (`_run_cycle` L1368–1456)

```
_resolve_scan_batch → _prefilter_batch → _load_universe_frames
→ _relative_strength_percentiles → _order_flow_batch
→ por símbolo: _evaluate_symbol_timeframes
→ _dynamic_selection_threshold → filtrar por umbral dinámico
→ ordenar y truncar a max_candidates → reporte
```

---

## 8. Archivos fuera de `scanner/` pero que gobiernan el escáner

- **`config/settings.py`:** todas las `scanner_*` y flags `ATLAS_SCANNER_INCLUDE_*` (sección L209–239 y validación L449–464).
- **`data/feed.py`:** usado como `MarketFeed` (no auditado en este documento).

---

*Fin del informe. Para cambiar umbrales, usar variables de entorno documentadas en `settings.py` o `update_config` en `OpportunityScannerService` (L572–621).*
