# ATLAS Quant - Exit Governance Audit

- Date: `2026-03-28`
- Scope: `exit_governance`
- Status: `active_focus`
- Dictamen operativo: `structured recommendations added, automated closing still pending`

## Executive Summary

ATLAS ya podia detectar posiciones deterioradas en `position_management`, pero aun faltaba convertir esa evidencia en recomendaciones de salida consistentes. La brecha real no era ver el problema, sino decidir de forma repetible si una posicion merece `exit_now`, `de_risk`, `take_profit` o simplemente `hold`.

La correccion implantada no automatiza cierres todavia. Primero formaliza la logica: salidas por invalidacion dura, por time-stop, por proteccion de beneficio y por exceso de concentracion. Con eso, el sistema deja de depender de lectura manual del libro para priorizar la accion.

## Internal Diagnosis

Hallazgos internos:

- `position_management` ya produce `open_r_multiple`, `holding_hours`, `thesis_drift_pct` y `symbol_heat_pct`.
- Esa evidencia no estaba traducida a una decision de salida o reduccion.
- Sin traduccion, el sistema seguia sabiendo que algo estaba mal pero sin dejar criterio reutilizable de accion.
- La salida seguia demasiado cerca de `post_mortem` y demasiado lejos de la operacion viva.

## External Confrontation

Referencias usadas para esta etapa:

- Fidelity documenta el uso disciplinado de `stop loss`, `stop limit` y `trailing stops`, lo que refuerza que la salida debe ser una politica estructurada y no una reaccion improvisada. Fuente: <https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/active-trader/things-to-know.pdf>
- El material de CME sobre tamano de posicion sigue siendo relevante aqui porque una salida parcial o reduccion de riesgo es una extension directa de la disciplina de riesgo por trade. Fuente: <https://www.cmegroup.com/education/courses/trade-and-risk-management/the-2-percent-rule.html>
- Investor.gov refuerza la idea de no confundir cantidad de posiciones con diversificacion real; eso sustenta decisiones de `de_risk` por concentracion. Fuente: <https://www.investor.gov/sites/investorgov/files/2019-02/Beginners-Guide-to-Asset-Allocation.pdf>

## Corrective Action Implemented

Se implanto un snapshot estructurado de `exit_governance` con:

- `recommendation`: `exit_now`, `de_risk`, `take_profit` u `hold`
- `exit_reason`: `hard_stop_loss_r`, `thesis_invalidated`, `time_stop`, `profit_target`, `book_concentration` o `protect_open_profit`
- `urgency`: `high`, `medium` o `low`
- resumen agregado de cuantos casos ya cumplen criterio de salida o reduccion

Este bloque ya queda visible en `operation/status` y reutilizable desde la fachada de `JournalPro`.

## What This Solves Now

- Prioriza la accion sobre las posiciones abiertas mas fragiles.
- Separa recorte parcial de cierre completo.
- Deja preparado el puente entre gestion viva del libro y futura gobernanza automatizada de salidas.

## What Still Remains

- Todavia no se ejecutan cierres automaticamente.
- Falta medir eficiencia real de las salidas sobre trades cerrados por tipo de salida.
- Falta enlazar esta capa con `post_trade_learning` para medir si los criterios de salida mejoran payoff y drawdown.

## Go / No-Go For This Stage

- `GO` para usar `exit_governance` como capa formal de recomendacion operativa.
- `NO-GO` para promocionar salidas automaticas a live sin evidencia before-after y sin rollback operativo.
