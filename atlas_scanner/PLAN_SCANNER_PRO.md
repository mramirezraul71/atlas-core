# ATLAS Scanner Pro — Diseño de Referencia

## Rol del scanner en el sistema

El scanner es el **motor de inteligencia de entrada** del sistema ATLAS:

- Escanea el mercado (en tiempo casi real).
- Convierte datos crudos (precio, opciones, flujos, macro, contexto visual) en **señales objetivas y explicables**.
- Filtra ruido y entornos sin edge.
- Entrega al módulo de estrategias **candidatos con edge estadístico positivo**.

El scanner **NO ejecuta trades**: produce `CandidateOpportunity` que luego pasan a:
- estrategia,
- ejecución,
- gestión de riesgo,
- evaluación/backtest.

---

## Pilares obligatorios del scanner

### Pilar 1 — Universo dinámico

No trabaja con lista fija; filtra cada ciclo según:

- **Liquidez**:
  - `volume_20d` > umbral,
  - `bid_ask_spread` < umbral,
  - OI mínimo en las alas de la cadena (para opciones).
- **Tradability de opciones**:
  - DTE en rango operativo **0–45 días** (incluyendo 0DTE),
  - spreads razonables (ej. < 0.10–0.15),
  - OI/volumen mínimos por strike/expiry.
- **Exclusiones fuertes**:
  - símbolos con `event_risk=True` (earnings, FOMC, macro/geo binarios inminentes).

Objetivo: evitar iliquidez y slippage que destruyen el edge.

---

### Pilar 2 — Multi‑factor scoring multidimensional

Score total compuesto de varios sub‑scores independientes:

- **Volatilidad** (~40 %):
  - IV Rank (20/50/100 días) e IV Percentile.
  - VRP (IV – RV) en 5/10/20/60 días.
  - Term structure (slope, curvatura).
  - Skew (25Δ call/put).

- **Gamma / dealers** (~20 %):
  - net GEX por strike/expiry.
  - Gamma walls, zero‑gamma, call/put walls.
  - Gamma flip distance %.
  - Max pain distance.

- **Volumen, OI y order flow** (~15 %):
  - OI change 1d %.
  - Call/put volume ratio.
  - Volume imbalance.
  - Strikes con máximo OI.

- **Precio y tendencia multi‑timeframe** (~20 %):
  - ADX (14).
  - EMA alignment (ranging / trending up / trending down).
  - Breakout status (VWAP, volume profile POC, soportes/resistencias).
  - Confirmación 4H + Daily.

- **Macro y estacionalidad** (~5 %, multiplicador):
  - Régimen global (VIX term structure, risk‑on/off).
  - Calendario macro/earnings.
  - Día de semana / OPEX / estacionalidad.

Score típico:

- `vol_score`, `gamma_score`, `oi_score`, `price_score` en [0,1].
- `macro_factor` como multiplicador en [0,1]+.
- Pesos y thresholds definidos en `SCORING_CONFIG`, calibrados por backtest/walk‑forward.

---

### Pilar 3 — Datos de alta calidad + fallback

El scanner trabaja con datos robustos y tiene plan B.

- **Fuentes principales**:
  - OHLCV multi‑timeframe (1m, 5m, 15m, 1H, 4H, diario).
  - Cadenas de opciones con:
    - precios, IV, OI, volumen por strike/expiry.
  - Derivados: IV Rank, VRP, GEX, max pain, etc.

- **Fallback y calidad**:
  - Orden de proveedores: ej. OpenBB/Tradier → Polygon → yfinance (u otros).
  - Caché de datos de corto plazo (ej. 5 min).
  - Regla: si el proveedor principal falla o da datos sucios:
    - se usa fallback,
    - o se marca el símbolo como no fiable para ese ciclo, en vez de inventar valores.

---

### Pilar 4 — Reglas de contexto y contraindicaciones

El scanner **bloquea o degrada** señales cuando el entorno destruye el edge.

- `matches_context_strategy_map`:
  - mapa entre contexto (rango/tendencia, vol alta/baja, régimen macro) y estrategias permitidas (IRON_CONDOR, DEBIT_SPREAD, CALENDAR, etc.).

- `event_risk`:
  - earnings, FOMC, datos macro críticos, eventos geopolíticos (como el briefing SPX).
  - Puede bloquear directamente o degradar fuerte el score.

- `regime detector`:
  - `regime_id` (ej. `long_gamma_deep`, `short_gamma_high_vol`).
  - `macro_regime` (risk‑on, risk‑off, `event_risk_high`), basado en VIX, curva VIX, spreads, etc.

---

### Pilar 5 — Ranking + deduplicación + risk caps

Control de cantidad, variedad y exposición:

- Filtro de score:
  - solo pasan candidatos con `total_score` ≥ umbral (ej. 65–75 de 100).

- Caps:
  - número máximo de trades/día (ej. 5–8).
  - caps por régimen y estrategia (máx. 2 iron condors, 1 debit spread, etc.).

- Deduplicación:
  - evitar señales redundantes:
    - mismo símbolo con expiries/strikes demasiado solapados.

- Control de correlación y exposición:
  - no concentrar demasiados trades en activos muy correlacionados.
  - controlar delta/gamma/vega netas de la cartera de candidatos.

---

### Pilar 6 — Logging completo + feature snapshot

Cada candidato viene con información suficiente para explicar y reproducir la decisión:

- `entry_reason`: resumen estructurado de motivos (no texto libre caótico).
- `feature_snapshot`: IV Rank/VRP, GEX, OI, ADX, patrones de vela, niveles clave, etc.
- `regime_id` y `macro_regime`.
- `score` y `component_scores`.

Uso:
- debugging en tiempo real,
- backtest y mejora continua del edge.

---

### Pilar 7 — Backtesting integrado y walk‑forward

El scanner debe ser backtesteable “tal cual”.

- Modo histórico:
  - recorrer datos históricos día/barra a barra,
  - recalcular features con las mismas reglas,
  - aplicar universo → filtros → scoring → contexto → caps.

- Logging histórico:
  - guardar `feature_snapshot`, `scores`, `regime_id`, `entry_reason`, etc., por señal.

- Conexión con módulo de simulación:
  - aplicar la estrategia elegida (iron condor, debit spread, etc.),
  - medir P&L, drawdowns, hit rate, ratios de riesgo.

- Walk‑forward:
  - calibrar `SCORING_CONFIG` por ventanas temporales,
  - validar en out‑of‑sample,
  - actualizar pesos/thresholds de forma controlada.

---

## Modelos principales

### `MarketContextSnapshot`

Resume el briefing cuantitativo por símbolo/índice:

- Datos de fecha/hora y sesión (ej. `0DTE Monday`, `post-OPEX roll`).
- Estructura de gamma:
  - flips (agg, semanal), distancias,
  - call/put walls,
  - max pain.
- Mapa de niveles:
  - spot,
  - resistencias,
  - soportes,
  - expected move 1σ (rango en puntos/precio).
- Macro:
  - eventos del día con hora/impacto (Retail Sales, FOMC, geopolítica),
  - resumen semanal.
- Régimen:
  - `regime_id`,
  - `macro_regime`.

### `SessionParams`

Config dinámica de la sesión:

- `market_hours_gate` (ej. esperar X minutos tras macro).
- `max_open_positions_recommended`.
- `score_min`.
- `bias` (long/neutral/defensive).
- `stop_estructural_intraday` (niveles que apagan nuevas entradas).
- Flags del sistema (modo auto, fail_safe, etc.).

### `CandidateOpportunity`

Lo que consume el módulo de estrategias:

- Símbolo + timestamp + tipo de activo.
- Contexto de opciones:
  - expiries candidatos y recomendados (DTE 0–45 días),
  - strikes relevantes (alas, call/put walls, strikes ATM/ITM/OTM).
- Features clave:
  - volatilidad (IV Rank/VRP/term structure/skew),
  - gamma/GEX,
  - OI/flow,
  - precio/tendencia (ADX, EMAs, patrones, VWAP/POC),
  - macro/regime.
- Scoring:
  - `total_score`,
  - `component_scores` por factor,
  - `weights` efectivos.
- Estrategia:
  - lista de estrategias recomendadas con prioridad,
  - compatibilidad con `matches_context_strategy_map`.
- Explainability:
  - `strengths`, `penalties`,
  - `explanation` (Pros/Risks),
  - `entry_reason` específico para la estrategia que se vaya a usar.

---

## Capas del scanner

1. **Universo + datos**  
   Filtro de liquidez/tradability + integraciones con datos (OpenBB, brokers, etc.) + fallback y caché.

2. **Features de mercado**  
   Módulos de features por categoría (vol, gamma, OI, precio, macro, señales visuales/cámara).

3. **Context snapshot**  
   Construye `MarketContextSnapshot` (gamma structure, mapa de niveles, expected move, eventos, escenarios A/B/C).

4. **Scoring multi‑factor**  
   Aplica `SCORING_CONFIG` sobre features → `component_scores` → `total_score`.

5. **Contexto y caps**  
   `matches_context_strategy_map`, event_risk, regime, caps por número y tipo de señales, deduplicación, correlación, exposición.

6. **Candidates y handoff**  
   Construye `CandidateOpportunity` para cada símbolo que pasa filtros, listo para el módulo de estrategia.

7. **Logging y backtest**  
   `feature_snapshot` + logging estructurado, modo histórico, walk‑forward.

---

## Extensibilidad

- Modelos con campos obligatorios + espacio (`meta`) para nuevas señales.
- `SCORING_CONFIG` como punto central para añadir nuevos factores sin romper código.
- Features encapsulados en módulos, para poder integrar nuevas fuentes (datos web, lectura de gráficos/cámara, etc.) sin romper contratos existentes.

**Este documento es la guía de diseño:** cada nueva fase (S4.x, S5.x, …) debe referenciar explícitamente qué partes de este plan implementa o extiende.
