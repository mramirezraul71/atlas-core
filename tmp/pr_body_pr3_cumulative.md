## Summary

Esta rama (`variante/nueva` → `intent-input-rename`) consolida en un solo hilo de revisión **varios parches ya entregados en commits separados**, con mensajes y alcances distinguibles:

- **Primera ola — paper safety:** cierre de hallazgos críticos de auditoría (mercado, límites de exposición, gobernanza de salidas en paper, reconciliación post-timeout de órdenes).
- **Segunda ola — riesgo y runtime en paper-first:** sizing Kelly conservador para equity, capa opcional y degradable de `MLSignalRanker` en runtime (commit mínimo) más enriquecimiento de contexto en el scanner (commit aparte), e integración de `market_open_config.json` como fuente de verdad operativa para parámetros de riesgo y schedule vía `settings`.

**Live:** la línea base sigue orientada a **paper / sandbox**; **live no queda habilitado** por estos cambios (Kelly live, ranker live, etc. siguen en modo conservador u opt-in explícito según lo ya mergeado en la rama).

## Included patches

### Paper safety patch

- Gate de **horario de mercado** y guard de **`max_open_positions`** en el flujo de auto-cycle / entrada.
- **Exit governance** en paper y reconciliación **post-timeout** de `broker_order_id` donde aplica el contrato de auditoría.
- **No** sustituye la revisión humana del riesgo de negocio ni habilita live por sí solo.

### Conservative Kelly sizing for equity

- Sizing Kelly para equity en modo **paper-first**, con límites de riesgo y caps alineados a la política conservadora ya acordada.
- **No** reescribe el motor genérico de Kelly fuera del alcance equity/paper ya integrado; **no** toca el paper safety patch anterior salvo coexistencia vía `settings`.

### MLSignalRanker runtime + scanner context enrichment

- **Runtime mínimo:** flags, `ml_score` / `ml_ranker_reason`, reordenación conservadora `(selection_score, ml_score)`, degradación limpia.
- **Enriquecimiento:** campos adicionales en el payload del candidato del scanner para alimentar mejor el `SignalContext` del ranker.
- **No** introduce veto duro ni cambia umbrales de selección del scanner fuera de lo ya revisado; **no** modifica el paper safety ni el sizing Kelly en archivos fuera del alcance acordado.

### market_open_config.json risk integration

- `market_open_config.json` como fuente de verdad para **riesgo y schedule** resuelta en `TradingConfig` con precedencia **env explícita > JSON > defaults**, validación por campo y fallbacks con `logger.warning`.
- Observabilidad: snapshot `market_open_operational` en readiness (fast + diagnostic); default de `max_risk_per_trade_pct` en normalización de estado del operation center alineado a `settings` en lugar de un literal fijo.
- **No** cambia la semántica del market hours gate ni la lógica del guard `max_open_positions`; **no** altera la implementación de Kelly en `kelly_sizer` / `signal_executor` más allá de leer los mismos campos de `settings` desde una configuración centralizada.

## Testing

Validación ya ejecutada en el trabajo de la rama (agrupada por bloque; resultados tal como se reportaron en el repo):

- **Paper safety / auto-cycle:**  
  `python -m pytest atlas_code_quant/tests/test_market_open_config_runtime.py atlas_code_quant/tests/test_auto_cycle_audit_fixes.py -q` — **9 passed** en la corrida documentada.

- **Kelly equity / operation center:**  
  `python -m pytest atlas_code_quant/tests/test_equity_sizing_kelly.py atlas_code_quant/tests/test_operation_center_core_contract.py -q` — **9 passed** en la corrida documentada.

- **ML runtime:**  
  `python -m pytest atlas_code_quant/tests/test_ml_signal_ranker_runtime.py -q` — **4 passed** (según el mensaje de commit del split ML).

- **market_open_config:**  
  `test_market_open_config_runtime.py` — carga de riesgo, fallbacks, guard de `max_positions`, fracción Kelly vía `settings`, JSON ausente, y precedencia env vs JSON.

## Risks

- La rama **acumula varios patches**; el riesgo principal pasa a ser **revisión de alcance y coherencia del conjunto**, no "mezcla silenciosa" en un solo commit gigante: los entregables están **separados en commits** con mensajes explícitos.
- Cambios en **JSON/env** pueden mover límites efectivos de riesgo sin tocar el código de gates; operadores deben revisar `market_open_config.json` y variables documentadas.
- **Live** no está objetivo de este PR; habilitar live sigue siendo decisión posterior con readiness y follow-ups.
- Dependencia de **orden de merge** con `intent-input-rename`: conviene revisar conflictos y duplicación de config al integrar.

## Follow-ups

- Integrar **self-audit** al ciclo operativo donde el protocolo de auditoría lo exija.
- **Atlas core** canónico: reproducibilidad y publicación/contrato claros entre repos.
- Gobernanza adicional coherente con auditoría (p. ej. **aplicación operativa** de `daily_loss_limit` / circuit breakers si el informe lo prioriza frente a solo centralizar el valor en `settings`).
- Documentación operativa: tabla única env + JSON + defaults para operadores.
- Evaluación formal de **live** solo después de los puntos anteriores y criterios de readiness ya definidos en el proyecto.
