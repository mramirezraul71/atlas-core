# ATLAS Scanner Phases S4-S5

Documento de continuidad de `ATLAS_SCANNER_PHASES.md` para las fases S4 y S5 en la rama `feat/atlas-scanner-s1-offline`.

Alcance de este documento:
- Reflejar solo estado implementado en código.
- Separar claramente lo implementado de lo diferido.
- Dejar trazabilidad operativa de arquitectura, contratos y decisiones.

---

## S4 Overview

Objetivo de S4: pasar de un scanner offline base a un scoring multifactor con features especializadas y explicabilidad por componente.

Estado general S4: **Implemented**.

### S4.1 Volatility features and scoring

**Implemented**
- Capa de features puras de volatilidad en `atlas_scanner/features/volatility.py`:
  - `compute_iv_rank(...)`
  - `compute_iv_percentile(...)`
  - `compute_vrp(...)`
  - `compute_vrp_series(...)`
- Builder de features en `atlas_scanner/features/builders.py`:
  - `build_vol_features(...)`
- Integracion en scoring offline:
  - `compute_vol_score(...)` en `atlas_scanner/scoring/offline.py`.

**Design decision**
- Separar calculo de features (determinista y reusable) de combinacion de score.

**Out of scope (S4.1)**
- Term structure y skew completos en formula de score.

### S4.2 Gamma features and scoring

**Implemented**
- Capa de features puras de gamma en `atlas_scanner/features/gamma.py`:
  - `find_call_wall(...)`
  - `find_put_wall(...)`
  - `find_gamma_flip(...)`
  - `classify_gamma_regime(...)`
- Builder de gamma en `atlas_scanner/features/builders.py`:
  - `build_gamma_features(...)`
- Integracion en scoring offline:
  - `compute_gamma_score(...)`.

**Design decision**
- Modelar estructura gamma con entidades explicitas (`StrikeGamma`, `GammaFeatures`) y regime discreto (`positive/neutral/negative`).

**Out of scope (S4.2)**
- Distancias a flip/max pain como factores ponderados completos en score.

### S4.3 OI/flow scoring

**Implemented**
- Scoring OI/flow en `compute_oi_flow_score(...)` con senales:
  - `oi_change_1d_pct`
  - `call_put_volume_ratio`
  - `volume_imbalance` (con derivacion desde `call_volume`/`put_volume` cuando aplica).
- Componente integrado al agregador multifactor.

**Design decision**
- Mantener semantica 0-100 consistente con otros componentes para facilitar agregacion/explicabilidad.

**Out of scope (S4.3)**
- Ranking por strikes OI dominantes como factor cuantitativo adicional.

### S4.4 Multifactor score explainability

**Implemented**
- Modelo `ComponentExplanation` en `atlas_scanner/scoring/offline.py`.
- `ScoredSymbol` extendido con:
  - `component_explanations`
  - `top_reasons`
- Explicadores por componente:
  - `explain_vol_component(...)`
  - `explain_gamma_component(...)`
  - `explain_price_component(...)`
  - `explain_macro_component(...)`
  - `explain_oi_flow_component(...)`
- Resumen global:
  - `summarize_score_explanation(...)`.
- Estados de componente:
  - `positive`
  - `neutral`
  - `negative`
  - `unavailable`.

**Design decision**
- Explainability estructurada y estable para consumo downstream; no texto libre como unico artefacto.

**Out of scope (S4.4)**
- Taxonomia avanzada de explicabilidad por estrategia.

---

## S5 Overview

Objetivo de S5: cerrar la capa de providers (ports/adapters) y robustecer el runner offline para operar en modo fail-soft con observabilidad.

Estado general S5: **Implemented** hasta S5.3.

### S5.0 Vol/Macro provider port and OpenBB adapter

**Implemented**
- Puerto `VolMacroProvider` en `atlas_scanner/ports/vol_macro_provider.py`.
- Contratos asociados:
  - `VolData`
  - `MacroData`
- Adapter `OpenBBVolMacroProvider` en `atlas_scanner/data/openbb_vol_macro.py`.
- Integracion en `run_offline_scan(...)` para enriquecer `SymbolSnapshot.meta` con:
  - `iv_history`, `iv_current`, `rv_annualized`
  - `vix`, `macro_regime`, `seasonal_factor`.

**Design decision**
- Ports/adapters para desacoplar dominio scanner de SDK externo.

**Out of scope (S5.0)**
- Multiples vendors Vol/Macro con arbitration dinamico.

### S5.1 Gamma/OI provider port and dummy adapter

**Implemented**
- Puerto `GammaOIProvider` en `atlas_scanner/ports/gamma_oi_provider.py`.
- Contratos asociados:
  - `GammaData`
  - `OIFlowData`
  - `StrikeGammaData`
- Adapter `DummyGammaOIProvider` en `atlas_scanner/data/dummy_gamma_oi.py`.
- Integracion en `run_offline_scan(...)` para inyectar:
  - `strike_gamma`/`gamma_strikes`
  - `net_gex`/`net_gamma`
  - `oi_change_1d_pct`, `call_put_volume_ratio`, `volume_imbalance`, `call_volume`, `put_volume`.

**Design decision**
- Introducir adapter dummy como baseline deterministico para pruebas e integracion progresiva.

**Out of scope (S5.1)**
- Provider real de gamma/OI de mercado en produccion.

### S5.2 Provider hardening and observability

**Implemented**
- Resolucion centralizada en `atlas_scanner/runner/provider_resolution.py`:
  - `resolve_offline_providers(...)`
  - `OfflineProviders`
  - `is_empty_*_data(...)`.
- Hardening en `run_offline_scan(...)`:
  - manejo `try/except` por llamada de provider,
  - comportamiento fail-soft (empty payload en error),
  - conteo de llamadas por estado (`ok`/`empty`/`error`).
- Observabilidad:
  - `provider_status` por simbolo dentro de `SymbolSnapshot.meta`,
  - metadatos agregados en `OfflineScanResult.meta["providers"]`.

**Design decision**
- No romper pipeline por error de un provider; degradar y continuar con trazabilidad explicita.

**Out of scope (S5.2)**
- Retry policy avanzada o circuit breaker configurable.

### S5.3 Price/Macro/Flow feature extraction refactor

**Implemented**
- Extraccion de logica feature-level desde scoring hacia `atlas_scanner/features/*`:
  - `features/price.py`:
    - `normalize_adx(...)`
    - `interpret_trend_state(...)`
    - `normalize_distance_to_vwap(...)`
  - `features/macro.py`:
    - `classify_macro_regime_from_vix(...)`
    - `score_macro_regime(...)`
    - `normalize_event_risk(...)`
    - `score_vix_bucket(...)`
    - `normalize_seasonal_factor(...)`
  - `features/flow.py`:
    - `normalize_oi_change(...)`
    - `normalize_call_put_volume_ratio(...)`
    - `resolve_volume_imbalance(...)`
    - `normalize_volume_imbalance(...)`
- `compute_price_score(...)`, `compute_macro_score(...)` y `compute_oi_flow_score(...)` ahora delegan normalizacion/derivacion en features.
- Contrato externo de scoring sin cambios (firmas y outputs estables).

**Design decision**
- Consolidar patron ya usado en vol/gamma: features puras + scoring como combinacion.

**Out of scope (S5.3)**
- Cambio de pesos/formulas de negocio en scoring.

---

## Current Architecture State (Implemented)

### Features layer
- Vive en `atlas_scanner/features/`.
- Features puras activas por dominio:
  - vol (`volatility.py`)
  - gamma (`gamma.py`)
  - price (`price.py`)
  - macro (`macro.py`)
  - flow (`flow.py`)
- Builders en `features/builders.py` para materializar `VolFeatures` y `GammaFeatures` y enriquecer `ProCandidateOpportunity`.

### Scoring layer
- Vive en `atlas_scanner/scoring/offline.py`.
- Componentes implementados:
  - `compute_vol_score(...)`
  - `compute_gamma_score(...)`
  - `compute_price_score(...)`
  - `compute_macro_score(...)`
  - `compute_oi_flow_score(...)`
- Agregacion multifactor:
  - `compute_component_weighted_score(...)`
  - renormalizacion por componentes disponibles.
- Compatibilidad con base legacy:
  - `score_symbol(...)` mantiene score base (liquidity/price/event_risk/spread) y aplica fallback cuando no hay senales multifactor disponibles.

### Explainability propagation
- `ScoredSymbol` expone:
  - `component_scores`
  - `component_explanations`
  - `top_reasons`
  - `strengths` y `penalties`
  - `explanation`
- `run_offline_scan(...)` transforma `ScoredSymbol` en `ProCandidateOpportunity` y propaga explainability en `candidate.meta` (`component_explanations`, `top_reasons`).

### Ports and adapters
- Puertos en `atlas_scanner/ports/`:
  - `VolMacroProvider`
  - `GammaOIProvider`
- Adapters en `atlas_scanner/data/`:
  - `OpenBBVolMacroProvider`
  - `DummyGammaOIProvider`

### Runner integration
- `run_offline_scan(...)`:
  - resuelve providers con `resolve_offline_providers(...)`,
  - enriquece `SymbolSnapshot.meta`,
  - rankea con `rank_symbols(...)`,
  - construye `candidate_opportunities` enriquecidas.
- `OfflineScanResult` expone:
  - `config`
  - `reference_datetime`
  - `selected_symbols`
  - `ranked_symbols`
  - `candidate_opportunities`
  - `universe_name`
  - `data_source_path`
  - `meta`.
- `meta` incluye observabilidad de providers:
  - estado agregado por provider (`ok`/`empty`/`error`)
  - conteos de llamadas por tipo de dato
  - filtros aplicados.

---

## Contracts and Visible Outcomes

**Implemented**
- API offline estable en `atlas_scanner/api/offline.py` (`scan`, `scan_top`, `scan_and_explain`).
- `scan_and_explain(top_n=...)` recorta tanto `ranked_symbols` como `candidate_opportunities`.
- Contratos de salida compatibles con flujo actual del scanner offline y pruebas unitarias existentes.

**Deferred**
- Contrato de scoring ampliado con nuevos objetos de features para `price_features`, `macro_features`, `oi_flow_features` dentro de `ProCandidateOpportunity` (hoy no se materializan via builder en la misma profundidad que vol/gamma).

---

## Design Decisions Consolidated in S4-S5

- Separacion estricta entre:
  - computo de features,
  - combinacion de score,
  - adquisicion de datos (ports/adapters).
- Arquitectura fail-soft en runner: errores de providers no detienen el scan.
- Explainability como estructura de datos estable y no solo string ad-hoc.
- Renormalizacion de score multifactor por disponibilidad real de componentes.

---

## Known Gaps (Deferred / Not Yet Implemented)

- Adapter real de mercado para `GammaOIProvider` (mas alla de dummy).
- Mayor riqueza de senales de scoring respecto a `PLAN_SCANNER_PRO.md`, por ejemplo:
  - term structure y skew integrados plenamente en score,
  - distance-to-flip y max-pain como factores cuantitativos completos.
- Formalizacion de loop de backtest / walk-forward conectado al scanner operativo.
- Consolidacion adicional de mapeo de features hacia `ProCandidateOpportunity` para price/macro/flow con la misma profundidad de vol/gamma.
- Documentacion de fases posteriores (S6+) una vez se cierre vendor real Gamma/OI y calibracion cuantitativa.

---

## Next Recommended Phase

### S6 — Gamma/OI real provider + calibration baseline

Objetivo recomendado:
- Implementar un adapter real de `GammaOIProvider`.
- Mantener contratos actuales y fail-soft.
- Definir baseline de calibracion para pesos/thresholds con evidencia de backtest.

Entregables minimos sugeridos:
- Nuevo adapter productivo para `GammaOIProvider`.
- Suite de tests de integracion equivalente a Vol/Macro.
- Nota de calibracion inicial de `component_weights` y thresholds documentada.

