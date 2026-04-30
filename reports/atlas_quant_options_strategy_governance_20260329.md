# ATLAS Quant - Options Strategy Governance Audit

- Fecha: `2026-03-29`
- Foco: `options_strategy_governance`

## Diagnostico

ATLAS ya elegia estructuras de opciones con criterio mejor que un selector binario, pero seguia demasiado comprimido en verticales e `iron_condor`. Eso dejaba tres huecos claros:

- no separaba bien tesis de `time_spread` frente a direccion simple,
- no traducia de forma suficiente la `term structure` a calendarios o diagonales,
- y no distinguia con claridad entre una estructura direccional y una necesidad de cobertura real.

## Confrontacion Externa

Fuentes confrontadas:

- Schwab: `credit vs debit spreads` deben guiarse por volatilidad implicita y perfil de riesgo, no solo por direccion.
- Fidelity: `collar` y coberturas existen para proteger stock o ganancias, no como sustituto generico de una tesis direccional.
- OCC: la familia de estructuras importa porque cambia theta, vega, contexto de inventario y condiciones de uso.

## Cambios Aplicados

- Se normalizaron aliases de tesis hacia un lenguaje operativo mas claro:
  - `time_spread`
  - `neutral_income`
  - `range_bound`
  - `hedged_overlay`
- El selector ahora diferencia mejor entre:
  - `directional_debit`
  - `directional_credit`
  - `neutral_theta`
  - `term_structure_time_spread`
- Se activan:
  - `call_calendar_spread`
  - `put_calendar_spread`
  - `call_diagonal_debit_spread`
  - `put_diagonal_debit_spread`
  cuando la tesis es de tiempo/vega y la `term_structure_slope` lo soporta.
- La gobernanza ahora registra `preferred_but_unavailable_strategy` cuando la mejor idea humana seria una cobertura tipo `protective_put` o `collar`, pero ATLAS no tiene confirmado el contexto de stock a proteger.

## Criterio Profesional

El sistema queda mas serio cuando:

- no vende credito por defecto en cualquier IV alta,
- no fuerza una vertical si la tesis real es de estructura temporal,
- y no automatiza una cobertura si no sabe si realmente existe inventario subyacente.

## Siguiente Paso

Medir en paper:

- frecuencia de cada `strategy_family`,
- porcentaje de `time_spread` elegidos cuando la curva temporal esta empinada,
- cuantos casos caen en `preferred_but_unavailable_strategy`,
- y si la nueva taxonomia reduce selecciones fragiles frente a evento cercano o baja liquidez.
