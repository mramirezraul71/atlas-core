# ATLAS Quant - Position Management Audit

- Date: `2026-03-28`
- Scope: `position_management`
- Status: `active_focus`
- Dictamen operativo: `baseline observability added, decision governance still pending`

## Executive Summary

ATLAS ya tenia datos suficientes en el journal para auditar la vida real de las posiciones, pero no estaba convirtiendo esa evidencia en una lectura operativa del libro abierto. El problema no era solo abrir o ejecutar mal; tambien faltaba una capa que dijera con claridad que posiciones estan envejeciendo mal, cuales estan perdiendo demasiado en terminos de R y donde se esta concentrando el calor del libro.

La correccion aplicada en esta fase no toca el schema de runtime ni fuerza cierres automaticos todavia. Lo que hace es construir la capa de observacion seria que faltaba: `position_management` ahora expone hold time, `open_r_multiple`, deriva de tesis, concentracion por simbolo y una watchlist priorizada de posiciones deterioradas.

## Internal Diagnosis

Hallazgos internos principales:

- El journal ya persiste `win_rate_at_entry`, `current_win_rate_pct`, `unrealized_pnl`, `risk_at_entry`, `entry_notional`, `updated_at` y `last_synced_at`.
- Esa informacion no estaba sintetizada en `operation/status`, por lo que el deterioro del libro seguia disperso entre tabla, monitor y lectura manual.
- No existia una medida simple del libro como `symbol_heat_pct`, `stale_loser_count` o `adverse_positions_count`.
- Tampoco habia una watchlist viva para responder la pregunta importante: que posiciones deberiamos estar mirando primero antes de que el dano escale.

## External Confrontation

Referencias externas utilizadas para esta etapa:

- CME Group insiste en limitar el riesgo por trade y tratar el tamano de posicion como primera defensa de supervivencia operativa, no como detalle secundario. Eso respalda el uso de `risk_budget` y de umbrales por calor del libro. Fuente: <https://www.cmegroup.com/education/courses/trade-and-risk-management/the-2-percent-rule.html>
- Investor.gov subraya que la diversificacion no es solo entre clases de activos sino tambien dentro de cada categoria. Eso respalda medir concentracion por simbolo y no asumir que un libro de muchas posiciones esta realmente diversificado. Fuente: <https://www.investor.gov/sites/investorgov/files/2019-02/Beginners-Guide-to-Asset-Allocation.pdf>
- El trabajo sobre `volatility managed portfolios` ya incorporado en ATLAS refuerza que el control del riesgo vivo no debe descansar solo en la entrada, sino en el ajuste continuo de exposicion cuando el regimen empeora. Fuente: <https://conference.nber.org/confer/2016/LTAMs16/Moreira_Muir.pdf>

## Gaps vs Benchmark

- ATLAS no estaba midiendo de forma operativa el calor por simbolo o familia.
- No diferenciaba entre posicion abierta y tesis aun sana.
- No priorizaba stale losers frente a perdedores recientes y todavia recuperables.
- Carecia de una salida intermedia entre `todo bien` y `ya fue demasiado tarde`.

## Corrective Action Implemented

Se implanto un snapshot estructurado de `position_management` con:

- `open_r_multiple` por posicion
- `holding_hours`
- `thesis_drift_pct`
- `symbol_heat_pct`
- concentracion por simbolo y por `strategy_type`
- alertas de `symbol_heat_exceeded`, `adverse_positions_present`, `stale_losers_present` y `thesis_drift_present`
- watchlist priorizada de posiciones problematicas

Este bloque ya queda visible en `operation/status` y reutilizable desde la fachada de `JournalPro`.

## What This Solves Now

- Permite detectar rapido donde el libro se esta pudriendo aunque la ejecucion ya haya pasado.
- Hace visible si el dano viene por concentracion, por deriva de tesis, por hold time excesivo o por perdida en R.
- Deja una base objetiva para el siguiente escalon: reglas de reduccion, recorte parcial, invalidacion y cierre.

## What Still Remains

- Todavia no hay accion automatica de reducir o cerrar; esta etapa deja primero la observacion viva.
- Falta conectar `position_management` con `exit_governance` para que la watchlist termine en decisiones formales.
- Falta medir `MAE/MFE` reales por trade cerrado en la capa de aprendizaje sin forzar cambios de schema al journal operativo.

## Go / No-Go For This Stage

- `GO` para usar `position_management` como capa viva de observacion y disciplina operativa.
- `NO-GO` para declarar gestion de posiciones madura hasta que la watchlist derive en reglas de reduccion/salida con evidencia before-after.
