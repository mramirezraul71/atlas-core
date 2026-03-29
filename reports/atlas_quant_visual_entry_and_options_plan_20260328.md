# ATLAS Quant - Visual Entry Optimization and Options Strategy Governance
**Fecha:** 2026-03-28  
**Branch:** `variante/nueva`  
**Estado:** fase siguiente preparada, no cerrada

## Resumen ejecutivo

ATLAS no estaba "sin capacidad visual"; estaba **partido en dos carriles**:

- el carril moderno de operacion (`api/main.py` -> `operation_center.py` -> `scripts/autonomous_loop.py`) describia `chart_plan` y `camera_plan`, pero no los llevaba hasta un gate operativo serio;
- el carril legado (`chart_launcher.py` -> `atlas_quant_core.py`) si podia abrir charts, pero estaba desconectado del pipeline autonomo real.

Ademas, la capa de opciones existe, pero el criterio actual es todavia demasiado plano para llamarlo un framework robusto de seleccion de estrategia.

## Hallazgos internos

### 1. Apertura de graficos

- `selector/strategy_selector.py` ya genera `chart_plan` con URLs y timeframes.
- `operations/strategy_playbooks.py` ya asume una "mission" visual.
- `operations/operation_center.py` solo capturaba contexto visual; no ejecutaba la mision de charts.
- `scripts/autonomous_loop.py` seguia enviando `capture_context=False` y construyendo la orden desde el scanner, no desde la propuesta completa del selector.

### 2. Chart launcher huérfano

- `atlas_code_quant/chart_launcher.py` si abre TradingView, pero:
  - esta desacoplado del loop autonomo actual;
  - fuerza una operativa centrada en `5m`;
  - trabaja con maximo 4 pestañas;
  - no estaba sincronizado con `chart_plan` por simbolo y timeframe del selector.

### 3. Estrategias con opciones

- `execution/option_selector.py` decide casi solo con:
  - direccion
  - regimen
  - `iv_rank`
  - `iv_hv_ratio`
- Eso hoy alcanza para spreads y estructuras basicas, pero no para una gobernanza seria de:
  - debit vs credit por contexto de volatilidad
  - estructuras neutrales de theta
  - calendars/diagonals
  - skew / term structure
  - eventos binarios
  - liquidez real por cadena/strike
  - sensibilidad theta/vega/gamma segun horizonte

## Que se implemento en esta fase

### 1. `chart_plan` y `camera_plan` ya viajan con la orden

Se extendio:

- `atlas_code_quant/api/schemas.py`
- `atlas_code_quant/selector/strategy_selector.py`

Ahora el `order_seed` del selector incluye:

- `chart_plan`
- `camera_plan`

Y esos campos pueden llegar de verdad a `OrderRequest`.

### 2. Nuevo `ChartExecutionService`

Se agrego:

- `atlas_code_quant/operations/chart_execution.py`

Funcion:

- ejecutar o preparar la "mision de chart" desde los targets del selector;
- abrir navegador con las URLs del `chart_plan` cuando el auto-open esta habilitado;
- devolver un estado estructurado de apertura y readiness visual.

Notas:

- el auto-open queda controlado por configuracion (`QUANT_CHART_AUTO_OPEN_ENABLED`);
- evita reabrir la misma mision dentro de una ventana corta (`QUANT_CHART_OPEN_COOLDOWN_SEC`).

### 3. Nuevo `visual_entry_gate`

Se integró en:

- `atlas_code_quant/operations/operation_center.py`

Ahora cada evaluacion puede devolver:

- `visual_entry_gate.status`
- `visual_entry_gate.degraded`
- `visual_entry_gate.chart_execution`
- `visual_entry_gate.context_capture_requested`
- `visual_entry_gate.context_capture_ok`

Este gate no vende una validacion visual falsa:

- si hay `chart_plan` pero no se abren charts, lo marca;
- si la camara es requerida pero `capture_context=False`, lo marca;
- si el proveedor visual no esta listo, lo marca.

### 4. El loop ya puede apoyarse en la propuesta del selector

Se extendio:

- `atlas_code_quant/api/main.py`
- `scripts/autonomous_loop.py`

Ahora ambos pueden apoyarse en `selector/proposal` para construir la orden con mas contexto y no solo desde el scanner bruto.

## Estado actual despues del cambio

ATLAS queda mejor preparado para la fase visual, pero **todavia no esta en una optimizacion visual completa de timing**.

Lo que ya si queda resuelto:

- el pipeline puede transportar la mision visual;
- la operacion puede medir si esa mision se atendio o se salto;
- el loop deja de estar tan ciego respecto al selector.

Lo que todavia falta:

1. Confirmar visualmente que el chart abierto coincide con simbolo y timeframe reales en pantalla, no solo con la URL solicitada.
2. Decidir si `camera_required` pasa a ser:
   - warning
   - gate estricto
3. Añadir taxonomia mas rica de opciones y conectar esa eleccion con la validacion visual.

## Proxima fase recomendada

### Fase A - endurecer la validacion visual

- convertir `visual_entry_gate` en criterio mas fuerte cuando `camera_plan.required=true`;
- verificar symbol/timeframe visibles;
- medir `chart_open_success_pct`, `visual_gate_pass_pct`, `context_capture_ok_pct`.

### Fase B - gobernanza seria de opciones

Añadir un marco por familias:

- direccional baja IV
- direccional alta IV
- neutral/theta
- volatilidad pura
- cobertura

Y evaluar con criterios adicionales:

- skew
- term structure
- DTE optimo por contexto
- chain liquidity real
- evento cercano
- sensibilidad theta/vega/gamma

## Benchmarks externos usados para la fase siguiente

- Fidelity Options hub: https://www.fidelity.com/learning-center/investment-products/options/options
- Charles Schwab on vertical spreads and volatility context: https://www.schwab.com/learn/story/bullish-bearish-vertical-options-spreads
- OCC Characteristics and Risks of Standardized Options: https://www.theocc.com/getContentAsset/42426879-3a15-4445-b66b-36b2c56ff3fc/dfc3d011-8f63-43f6-9ed8-4b444333a1d0/march_2022_riskstoc.pdf

## Dictamen

Esta fase deja a ATLAS en una posicion mucho mas seria para abordar:

- optimizacion visual de entrada
- gobierno de estrategias con opciones

Pero no conviene fingir cierre prematuro:

- la parte visual ya esta cableada, no consolidada;
- la parte de opciones ya esta diagnosticada, no re-modelada todavia.
