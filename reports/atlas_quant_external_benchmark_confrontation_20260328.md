# ATLAS Quant - Flujo de Confrontacion Web y Registro de Fuentes Externas

**Fecha:** 2026-03-28
**Repositorio:** `C:\ATLAS_PUSH`
**Objetivo:** dejar en ATLAS un flujo reusable para confrontar su comportamiento con referencias externas serias y reutilizar esas experiencias por etapa del trading.

---

## 1. Regla de uso

ATLAS no debe buscar en web por curiosidad desordenada.
Debe hacerlo cuando una etapa del proceso de trading:

- muestra degradacion persistente,
- no tiene explicacion causal suficiente con evidencia local,
- o necesita benchmark externo para decidir si una metrica, guardrail o criterio esta bien elegido.

---

## 2. Flujo de confrontacion web

Secuencia obligatoria:

1. detectar la pregunta concreta de la etapa;
2. elegir primero fuentes serias;
3. escanear metricas, guardrails, fallos tipicos y limites;
4. comparar esas fuentes contra lo que realmente hace ATLAS;
5. convertir el gap en acciones medibles;
6. persistir el aprendizaje y reutilizarlo en la siguiente etapa.

Prioridad de fuentes:

1. `papers_academicos`
2. `regulatory_guidance`
3. `broker_execution_docs`
4. `tool_or_feed_docs`

No usar opinion informal como benchmark principal.

---

## 3. Paginas y documentos ya visitados durante la auditoria

### 3.1 Scanner selection

- Time Series Momentum  
  https://elmwealth.com/wp-content/uploads/2017/06/timeseriesmomentum.pdf
- Momentum: Evidence and Insights 30 Years Late  
  https://papers.ssrn.com/sol3/papers.cfm?abstract_id=4602426
- Residual Momentum  
  https://repub.eur.nl/pub/22252/ResidualMomentum-2011.pdf
- Momentum Crashes  
  https://www.nber.org/papers/w20439.pdf
- Volatility Managed Portfolios  
  https://conference.nber.org/confer/2016/LTAMs16/Moreira_Muir.pdf
- Do Industries Explain Momentum?  
  https://spinup-000d1a-wp-offload-media.s3.amazonaws.com/faculty/wp-content/uploads/sites/3/2019/09/DoIndustriesExplainMomentum.pdf
- Short-Term Return Reversal: The Long and the Short of It  
  https://econen.sufe.edu.cn/_upload/article/files/d3/34/c6aa75294353b29e32728df8b04e/01c74230-970a-454a-8483-265efa7c6ba1.pdf
- Illiquidity and Stock Returns: Cross-Section and Time-Series Effects  
  https://w4.stern.nyu.edu/finance/docs/WP/2000/pdf/wpa00041.pdf
- yfinance legal information  
  https://ericpien.github.io/yfinance/getting_started/legal.html

### 3.2 Entry validation y execution quality

- FINRA Regulatory Notice 21-23  
  https://www.finra.org/sites/default/files/2021-06/Regulatory-Notice-21-23.pdf
- Transaction Cost Analysis and Implementation Shortfall  
  https://pages.stern.nyu.edu/~jhasbrou/STPP/drafts/STPPms13b.pdf
- Commitment to Execution Quality | Trading with Fidelity  
  https://www.fidelity.com/trading/execution-quality/overview

---

## 4. Como debe usar ATLAS estas fuentes

### 4.1 scanner_selection

Buscar:

- trayectoria persistente,
- momentum residual,
- crash risk,
- regimen de volatilidad,
- filtros de liquidez,
- limites del feed de datos.

### 4.2 entry_validation

Buscar:

- arrival price,
- coste de entrar,
- deriva desde precio de decision,
- spread,
- probabilidad de ejecucion,
- deterioro del edge por friccion.

### 4.3 execution_quality

Buscar:

- best execution factors,
- effective spread,
- price improvement,
- speed of execution,
- likelihood of execution,
- comparativa entre orden planeada y orden realmente enviada.

### 4.4 etapas futuras

El mismo flujo debe extenderse luego a:

- `position_management`
- `exit_governance`
- `post_trade_learning`

---

## 5. Salida esperada

Si ATLAS sigue este flujo, cada auditoria futura deberia dejar:

- la pregunta concreta de la etapa,
- las fuentes externas usadas,
- el gap entre benchmark y ATLAS,
- la correccion aplicada,
- el before/after,
- y la huella persistida en memoria/bitacora.
