# ATLAS Quant - Options Strategy Governance
**Fecha:** 2026-03-28  
**Branch:** `variante/nueva`

## Objetivo

Endurecer la seleccion de estrategias con opciones sin romper la base ejecutable actual.

## Diagnostico previo

El selector anterior elegia casi solo con:

- direccion
- regimen
- `iv_rank`
- `iv_hv_ratio`

Eso era util como heuristica inicial, pero insuficiente para una gobernanza seria. Faltaban criterios para distinguir:

- expansion vs compresion de volatilidad
- evento cercano vs contexto limpio
- premium selling definido vs debito direccional
- neutral theta vs direccional controlada
- liquidez fragil vs liquidez sana
- skew y term structure como señales de contexto, aunque todavia no como optimizador completo

## Cambios aplicados

Archivo principal:

- `atlas_code_quant/execution/option_selector.py`

Mejoras:

- `pick_strategy()` ahora acepta contexto adicional:
  - `thesis`
  - `event_near`
  - `liquidity_score`
  - `skew_pct`
  - `term_structure_slope`
  - `prefer_defined_risk`
- La decision ya no es solo "alta IV = credit / baja IV = long premium".
- Se introdujo `describe_strategy_governance()` para devolver una explicacion estructurada.
- `ContractSelection` ahora incluye `governance`.
- `select_option_contract()` ya persiste esa gobernanza junto con la estructura elegida.

## Politica actual

### Direccional con evento cercano

- se evita vender prima direccional de forma agresiva
- se favorece:
  - `long_call`
  - `long_put`
  - `bull_call_debit_spread`
  - `bear_put_debit_spread`

### Direccional con IV alta pero contexto limpio

- se permiten spreads de credito definidos:
  - `bull_put_credit_spread`
  - `bear_call_credit_spread`

### Neutral / theta

- `iron_condor` sigue siendo la estructura neutral por defecto
- `iron_butterfly` solo aparece cuando la tesis neutral es mas explicita y el contexto es favorable

### Liquidez debil

- se castiga premium selling direccional
- el selector deriva hacia estructuras de debito mas controlables

## Lo que todavia no se hace

Esta fase endurece la gobernanza, pero no pretende vender humo:

- no se modelan todavia calendars o diagonals en la seleccion ejecutable principal
- no se optimiza todavia por term structure real a nivel de expiraciones cruzadas
- no se resuelve todavia una taxonomia completa de hedge con underlying real (`covered_call`, `protective_put`)

## Pruebas

- `atlas_code_quant/tests/test_option_selector.py`
- `atlas_code_quant/tests/test_signal_generator_options.py`

## Benchmarks externos de referencia

- Fidelity options education hub: https://www.fidelity.com/learning-center/investment-products/options/options
- Schwab on vertical spreads and volatility context: https://www.schwab.com/learn/story/bullish-bearish-vertical-options-spreads
- OCC Characteristics and Risks of Standardized Options: https://www.theocc.com/getContentAsset/42426879-3a15-4445-b66b-36b2c56ff3fc/dfc3d011-8f63-43f6-9ed8-4b444333a1d0/march_2022_riskstoc.pdf

## Dictamen

ATLAS ya no esta eligiendo opciones con una logica tan plana como antes.  
La seleccion sigue siendo conservadora y compatible con la base actual, pero ahora:

- distingue mejor entre expansion y compresion
- penaliza eventos cercanos para premium selling
- incorpora una narrativa reutilizable de gobernanza
- deja lista la siguiente fase: taxonomia avanzada y estructuras de tiempo
