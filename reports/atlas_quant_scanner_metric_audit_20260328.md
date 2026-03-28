# ATLAS Quant - Auditoria de Metricas del Scanner

**Fecha:** 2026-03-28
**Repositorio:** `C:\ATLAS_PUSH`
**Fase activa del proceso:** `scanner_selection`
**Marco abierto:** el algoritmo de auto-auditoria queda preparado para extenderse a `entry_validation`, `execution_quality`, `position_management`, `exit_governance` y `post_trade_learning`

---

## 1. Dictamen ejecutivo

El scanner actual de ATLAS no esta conceptualmente mal orientado. Usa familias de señal defendibles:

- tendencia por EMA stack,
- breakout Donchian,
- pullback condicionado dentro de tendencia,
- fuerza relativa,
- confirmacion multi-timeframe,
- proxy de order flow.

El problema no es de "falta de indicadores", sino de calibracion del ranking:

1. sobrepremia `local_win_rate` y metricas tacticas recientes;
2. no mete suficiente peso a `expectancy`, `profit_factor`, `sample_size` y `avg_win / avg_loss`;
3. mezcla horizontes intradia y swing dentro del mismo ranking;
4. no incorpora bien filtros de crash regime, residual momentum ni contexto sectorial;
5. usa un `predicted_move_pct` que aproxima amplitud volatilidad-escalada, no trayectoria robusta.

---

## 2. Hallazgos de codigo

### 2.1 Formula actual

El `selection_score` combina:

- `local_win_rate_pct`
- `relative_strength_pct`
- `signal_strength_pct`
- `alignment_score`
- `evidence_score`
- `order_flow_score`
- `order_flow_alignment_score`
- `adaptive_bias`

La formula favorece demasiado la probabilidad local reciente y demasiado poco la calidad real del payoff.

### 2.2 Metricas calculadas pero infrautilizadas

El scanner calcula:

- `local_profit_factor`
- `local_sample`

pero hoy no las usa para ordenar ni vetar con suficiente fuerza. Eso deja pasar setups con hit rate aceptable y expectativa pobre.

### 2.3 Horizonte excesivamente corto para parte del score

El backtest local del scanner evalua horizontes muy cortos en varios timeframes. Esto es util como termometro tactico, pero no como criterio dominante de trayectoria. El riesgo es perseguir ruido intradia y rebotes falsos.

### 2.4 Falta de contexto estructural

No se observa en el scanner actual un control fuerte y explicito de:

- regimen de volatilidad,
- crash risk de momentum,
- momentum residual,
- confirmacion sectorial o industrial,
- breadth o salud del mercado,
- separacion formal entre motor intradia y motor swing.

### 2.5 Riesgo de feed

Para equities, el scanner depende de `yfinance` como fuente de OHLCV. Es aceptable para investigacion y prototipado, pero no es la base ideal para un scanner intradia serio con sensibilidad en `5m/15m/1h`.

---

## 3. Interpretacion operativa

La evidencia combinada sugiere que ATLAS esta eligiendo demasiados candidatos que "parecen bien" localmente, pero que no tienen suficiente confirmacion de trayectoria ni de expectativa ajustada por riesgo.

Eso explica un patron peligroso:

- win rate no desastroso,
- demasiadas posiciones negativas acumuladas,
- perdida media mas grande que ganancia media,
- dano concentrado en longs.

En ese contexto, el scanner puede estar actuando como amplificador de setups tacticos fragiles en lugar de actuar como filtro de calidad.

---

## 4. Criterios mas robustos que deben entrar

### P0 - Deben entrar ya al ranking o al veto

- `expectancy_pct` local por metodo y timeframe
- `profit_factor` local penalizado por muestra debil
- `avg_win / avg_loss`
- `sample_size` como peso o factor de confianza
- separacion explicita entre ranking swing e intradia

### P1 - Deben entrar al contexto

- momentum temporal `3m/6m/12m`
- control de reversión corta para no perseguir ruido
- confirmacion sectorial o industrial
- volatility regime
- crash-risk veto para longs de momentum

### P2 - Deben entrar a la gobernanza

- seguimiento antes/despues de cada ajuste
- reporte diario de degradacion del scanner
- memoria persistente de criterios que mejoran o empeoran el libro

---

## 5. Algoritmo logico abierto para ATLAS

El algoritmo general queda abierto a todo el proceso de trading, con foco actual en `scanner_selection`.

Secuencia:

1. detectar degradacion
2. auditar metricas internas
3. buscar referencias externas
4. comparar interno vs externo
5. diagnosticar causa raiz
6. corregir pesos, vetos y separacion de motores
7. seguir resultados antes/despues
8. persistir aprendizaje en memoria y bitacora
9. promover politica solo si la evidencia se sostiene

---

## 6. Resultado esperado del rediseño

Si el scanner se recalibra correctamente, ATLAS deberia:

- seleccionar menos candidatos, pero mas robustos,
- reducir bursts de entradas long fragiles,
- bajar la discrepancia entre win rate y expectativa real,
- dejar de sobrepremiar setups tacticos con payoff deficiente,
- aprender de forma acumulativa que metricas funcionan mejor por fase del proceso.

---

## 7. Soporte implementado en el repo

Se dejo un protocolo reusable en:

- `atlas_code_quant/learning/trading_self_audit_protocol.py`
- `atlas_code_quant/scripts/record_trading_self_audit_note.py`
- `reports/atlas_quant_trading_process_audit_framework_20260328.md`

El primero define el marco logico abierto del proceso de trading y marca `scanner_selection` como foco actual.
El segundo permite persistir ese protocolo en memoria/bitacora usando el bridge existente de Quant.
El tercero deja fijado el mismo esquema de auditoria para todas las etapas futuras del proceso de trading.
La fase siguiente ya activada bajo ese mismo marco es `entry_validation`, documentada en `reports/atlas_quant_entry_validation_audit_20260328.md`.

---

## 8. Recalibracion aplicada

La auditoria ya fue traducida a cambios concretos del scanner.

Cambios implementados:

- se agregaron `min_local_profit_factor` y `min_backtest_sample` como gates nuevos;
- el backtest local ahora expone:
  - `expectancy_pct`
  - `avg_win_pct`
  - `avg_loss_pct`
  - `payoff_ratio`
  - `sample_confidence_pct`
  - `quality_score_pct`
- la eleccion del mejor metodo dentro del timeframe ya no prioriza solo `local_win_rate`;
- `selection_score` ahora prioriza `local_quality_score_pct` por encima del mero acierto local;
- los candidatos aceptados y rechazados ahora cargan mas contexto de calidad local.

Archivos implementados:

- `atlas_code_quant/config/settings.py`
- `atlas_code_quant/scanner/opportunity_scanner.py`
- `atlas_code_quant/tests/test_scanner_metric_recalibration.py`

Validacion ejecutada:

- `python -m pytest -q atlas_code_quant/tests/test_scanner_metric_recalibration.py atlas_code_quant/tests/test_operation_center_status.py atlas_code_quant/tests/test_operation_center_autonomous_guards.py`
- resultado: `8 passed`
