## Summary

Esta rama (`variante/nueva` → `intent-input-rename`) consolida en un solo hilo de revisión **todas las olas de remediación de auditoría aplicadas hasta ahora**:

- **Primera ola — paper safety:** cierre de hallazgos críticos de auditoría (mercado, límites de exposición, gobernanza de salidas en paper, reconciliación post-timeout de órdenes).
- **Segunda ola — riesgo y runtime en paper-first:** sizing Kelly conservador para equity, capa opcional y degradable de `MLSignalRanker` en runtime (commit mínimo) más enriquecimiento de contexto en el scanner, integración de `market_open_config.json` como fuente de verdad operativa para parámetros de riesgo y schedule vía `settings`, documentación/manifiesto canónico de `atlas_core` y scaffolding de reproducibilidad, e integración de un self-audit operacional en paper.

**Live:** la línea base sigue orientada a **paper / sandbox**; **live no queda habilitado** por estos cambios (Kelly live, ranker live, self-audit con semántica BLOCK, etc. siguen en modo conservador u opt-in explícito según lo ya definido).

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

### Canonical atlas_core documentation and reproducibility

- `atlas_core/CANONICAL.md` documenta el layout canónico de `atlas_core` y su relación con el shim `atlas_code_quant.atlas_core`.
- `atlas_core/MANIFEST.json` + `scripts/verify_atlas_core_manifest.py` aseguran que los módulos core listados puedan importarse sin errores, con tests dedicados en `test_atlas_core_canonical_manifest.py`.
- No cambia lógica de trading; es scaffolding de documentación y verificación.

### Operational self-audit (paper-first)

- Runner `operational_self_audit` integrado en puntos definidos del ciclo de paper (vía readiness), produciendo un `SelfAuditResult` estructurado (severidad, checks, scope).
- El self-audit es **read-only** respecto a gates/submit en paper: observa, agrega un resumen compacto en readiness cuando está habilitado, y no introduce nuevos vetos duros.
- No modifica execution, journal, Kelly, ML ni market_open_config; se apoya en la configuración y snapshots ya existentes.

## Testing

(Resumen de la validación ya ejecutada a lo largo de la rama, tal como se documentó en commits y PRs individuales; puedes mantener los comandos ya listados y añadir solo las nuevas suites si aún no estaban mencionadas.)

- Tests de paper safety / auto-cycle.
- Tests de Kelly equity sizing y operation center.
- Tests de ML runtime (`test_ml_signal_ranker_runtime.py`).
- Tests de market_open_config (`test_market_open_config_runtime.py`).
- Tests de atlas_core manifest (`test_atlas_core_canonical_manifest.py` + script `verify_atlas_core_manifest.py`).
- Tests de operational self-audit (`test_operational_self_audit.py`).

## Risks

- La rama **acumula varios patches**; el riesgo principal es de **revisión y gobernanza del conjunto** más que de cambios aislados.
- Cambios en JSON/env pueden modificar límites efectivos de riesgo; operadores deben revisar `market_open_config.json` y documentación de env vars.
- Self-audit, atlas_core manifest y ML runtime son scaffolding y observabilidad en paper; live sigue deshabilitado y requiere decisión explícita adicional.
- Conviene revisar conflictos y duplicaciones de config al integrar con `intent-input-rename`.

## Follow-ups

- Integrar el self-audit en flujos de reporting/monitoring (alertas, dashboards) una vez estabilizado.
- Endurecer semántica de self-audit (p. ej. permitir BLOCK) solo cuando live sea candidato real.
- Extender la integración de atlas_core canónico en CI (verify script en pipelines de release).
- Documentar de forma única (tabla) la matriz env + JSON + defaults para operadores.
- Evaluar formalmente habilitación live únicamente después de revisar este conjunto completo más los criterios de readiness acordados.
