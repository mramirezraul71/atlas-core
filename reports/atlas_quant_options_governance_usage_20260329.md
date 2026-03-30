# ATLAS Quant - Options Governance Usage Baseline

- Fecha: `2026-03-29`
- Scope auditado: `paper`
- Fuente: `atlas_code_quant/data/journal/trading_journal.sqlite3`

## Dictamen

La gobernanza de estrategias con opciones ya esta implementada en el selector, en `operation/status`, en el scorecard y en observabilidad. Sin embargo, en el journal operativo `paper` aun no existe evidencia de uso real de esa capa.

## Snapshot actual

- `option_entries_total`: `0`
- `open_option_positions`: `0`
- `closed_option_trades`: `0`
- `time_spread_count`: `0`
- `vertical_count`: `0`
- `neutral_theta_count`: `0`
- `single_leg_count`: `0`
- `strategy_diversity_count`: `0`
- `time_spread_share_pct`: `0.0`
- `vertical_share_pct`: `0.0`

## Alerta activa

- `options_governance_not_exercised`
  - `Options governance is implemented, but no option trades exist yet in the journal for this account scope.`

## Interpretacion profesional

Esto significa que ATLAS ya tiene capacidad para:

- distinguir familias `directional_vertical`, `neutral_theta` y `term_structure_time_spread`
- proponer `calendar` y `diagonal` cuando la tesis y la term structure lo justifican
- exponer esa gobernanza en API, scorecard y dashboards

Pero todavia no tenemos muestra empirica para responder preguntas como:

- si ATLAS sigue abusando de verticales en paper real
- si empieza a usar `calendar`/`diagonal` cuando corresponde
- si la nueva taxonomia de opciones mejora la mezcla estructural del libro

## Siguiente criterio de seguimiento

La siguiente sesion paper con opciones debe medir al menos:

- nacimientos de estrategias de opciones por `strategy_type`
- mezcla por familia
- share real de `calendar/diagonal`
- concentracion en verticales
- diferencia between `selector capable` vs `selector actually used`

Hasta entonces, esta fase debe considerarse:

- `implemented`: si
- `observable`: si
- `operationally exercised`: no
