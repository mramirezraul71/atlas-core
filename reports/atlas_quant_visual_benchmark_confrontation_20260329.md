# ATLAS Quant - Visual Entry Benchmark Confrontation
**Fecha:** 2026-03-29  
**Branch:** `variante/nueva`  
**Estado:** benchmark visual integrado al protocolo de aprendizaje

## Objetivo

Persistir dentro de ATLAS un benchmark reusable sobre **como entra visualmente un operador humano disciplinado** y como debe traducirse eso a reglas automaticas medibles.

## Fuentes externas incorporadas

- Fidelity - Chart Patterns Transcript  
  https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/learning-center/Transcript_Chart%20patterns_v2.pdf
- Schwab - How to Read Stock Charts and Trading Patterns  
  https://www.schwab.com/learn/story/how-to-read-stock-charts-and-trading-patterns
- Schwab - Technical Indicators: 3 Trading Traps to Avoid  
  https://www.schwab.com/learn/story/technical-indicators-3-trading-traps-to-avoid
- Schwab - Using the Volume Profile Indicator  
  https://workplace.schwab.com/story/using-volume-profile-indicator
- CME Group - Trend Following with Managed Futures: Historical Perspectives  
  https://www.cmegroup.com/content/dam/cmegroup/education/files/trend-following-with-managed-futures-historical-perspectives.pdf
- Hasbrouck - Transaction Cost Analysis and Implementation Shortfall  
  https://pages.stern.nyu.edu/~jhasbrou/STPP/drafts/STPPms13b.pdf

## Sintesis profesional

El metodo visual mas robusto no es "mirar mas indicadores", sino operar con este orden:

1. contexto superior claro
2. nivel tecnico defendible
3. trigger de confirmacion
4. invalidacion explicita

En humanos competentes, la mejora visual viene de **esperar confirmacion** y aceptar entrar algo mas tarde si eso reduce trampas de breakout o pullbacks rotos.

En sistemas automaticos, la mejora no viene de copiar la intuicion humana, sino de traducirla a:

- reglas de confirmacion visual
- evidencia de captura util
- comparacion entre tesis esperada y OCR real
- benchmark de entrada y costo de ejecucion

## Traduccion a ATLAS

Este benchmark se incorporo al protocolo maestro para que ATLAS:

- detecte cuando una entrada necesita contraste visual serio
- confronte lo que su camara/OCR ve contra la tesis esperada
- busque en la web practicas comparables para ese fallo
- traduzca la comparacion a thresholds, tests y scorecard
- persista el aprendizaje en memoria y bitacora

## Criterios reutilizables

- `visual_readiness_score_pct`
- `visual_alignment_score_pct`
- `ocr_confidence_pct`
- `chart_symbol_match`
- `chart_timeframe_match`
- `capture_evidence_present`
- `visual_gate_block_rate`

## Bucle de retroalimentacion

ATLAS debe aplicar siempre este esquema:

1. detectar que setup visual genero mas bloqueos, dudas o degradacion
2. buscar benchmark externo especifico del patron o error
3. comparar lo que hace ATLAS contra la fuente
4. convertir esa comparacion en reglas o thresholds
5. medir si el cambio mejora la calidad de entrada
6. persistir el aprendizaje

## Dictamen

ATLAS ya no debe tratar estas webs como referencia puntual, sino como parte de su **memoria de confrontacion externa** para `entry_validation`, `visual_entry_optimization` y, cuando aplique, `options_strategy_governance`.
