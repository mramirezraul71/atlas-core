# ATLAS Quant - Post Trade Learning Audit

- Date: `2026-03-28`
- Scope: `post_trade_learning`
- Status: `active_focus`
- Dictamen operativo: `learning loop formalized, policy promotion still conservative`

## Executive Summary

ATLAS ya contaba con trades cerrados, post-mortem parciales y una capa adaptativa, pero faltaba una vista corta y operativa que resumiera que esta aprendiendo realmente del libro cerrado. El problema no era solo almacenar resultados, sino convertirlos en candidatos de politica y en una lectura reusable por estrategia y causa raiz.

La correccion implantada no cambia politicas automaticamente. Primero formaliza el aprendizaje: cobertura de post-mortem, desglose de causas raiz, aprendizaje por estrategia y candidatos de revision cuando una familia acumula dano repetido.

## Internal Diagnosis

Hallazgos internos:

- El journal ya persistia `post_mortem_json`, `post_mortem_text`, `realized_pnl` y `strategy_type`.
- Esa informacion no estaba resumida en una capa viva visible desde `operation/status`.
- El sistema podia recordar eventos, pero aun no mostraba con claridad que estrategias estaban pidiendo `review_reduce_or_disable`.
- Faltaba una forma medible de distinguir entre historico almacenado y aprendizaje utilizable.

## External Confrontation

Referencias usadas para esta etapa:

- CME resalta la disciplina del `trade log` y la revision posterior como parte de la gestion seria del riesgo y del proceso de mejora. Fuente: <https://www.cmegroup.com/education/courses/trade-and-risk-management/trade-log.html>
- Fidelity ya venia respaldando la necesidad de clasificar salidas y ordenes con criterio estructurado, lo que se conecta directamente con la calidad del aprendizaje posterior. Fuente: <https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/active-trader/things-to-know.pdf>

## Corrective Action Implemented

Se implanto un snapshot estructurado de `post_trade_learning` con:

- `closed_trades`
- `post_mortem_coverage_pct`
- `root_cause_breakdown`
- `strategy_learning`
- `policy_candidates`

Los candidatos de politica se etiquetan hoy de forma conservadora como `review_reduce_or_disable` cuando una estrategia acumula muestra cerrada suficiente y PnL agregado negativo.

## What This Solves Now

- Vuelve visible si ATLAS esta aprendiendo algo util o solo guardando cierres.
- Resume rapidamente donde se concentra el dano historico.
- Prepara el puente entre historial cerrado y futuras politicas con evidencia.

## What Still Remains

- Todavia no hay promocion automatica de politica.
- Falta comparar before-after cuando una politica candidata se aplique de verdad.
- Falta integrar esta capa con el `AdaptiveLearningService` de forma mas estrecha y con gobierno formal de rollback.

## Go / No-Go For This Stage

- `GO` para usar `post_trade_learning` como capa operativa de sintesis y priorizacion del aprendizaje.
- `NO-GO` para dejar que esta capa modifique reglas de produccion sin evidencia acumulada y criterios formales de promotion/revert.
