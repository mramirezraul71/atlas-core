# ATLAS Quant - Trading Implementation Scorecard

- Generated at: `2026-03-28T10:40:31.137945+00:00`
- Process compliance: `97.5/100` (healthy)
- Implementation usefulness: `20.0/100` (critical)

## Metrics

- `process_compliance_score`: `97.5/100` (healthy) - Medir cuanto del marco metodologico ya esta realmente implantado por etapa.
- `artifact_coverage_score`: `100.0/100` (healthy) - Medir si cada avance deja huella en protocolo, informes y archivos reutilizables.
- `memory_persistence_score`: `68.07/100` (watch) - Medir si ATLAS esta reteniendo los cambios en memoria y bitacora.
- `external_benchmark_coverage_score`: `100.0/100` (healthy) - Medir cuantas etapas auditadas estan respaldadas por confrontacion externa seria.
- `observability_feedback_score`: `93.33/100` (healthy) - Medir si Grafana, Prometheus y el chequeo operativo estan devolviendo feedback confiable para sostener disciplina.
- `test_guardrail_score`: `100.0/100` (healthy) - Medir cuanta proteccion automatizada existe sobre lo ya implantado.
- `implementation_usefulness_score`: `20.0/100` (critical) - Medir si ya existe evidencia operativa suficiente para afirmar que los cambios sirven de algo.

## Supporting Indicators

- `brain_delivery_ratio_pct`: `54.39`
- `attributed_open_positions_pct`: `0.0`
- `open_untracked_ratio_pct`: `100.0`
- `evidence_sufficiency_score`: `0.0`
- `external_learning_translation_score`: `100.0`
- `observability_feedback_score`: `93.33`
- `grafana_reload_resilience_pct`: `66.67`
- `grafana_alerting_ready_pct`: `100.0`

## Interpretation

- El proceso va por `97.5/100`: esto mide implantacion metodologica, no edge de mercado.
- La utilidad va por `20.0/100`: esto mide si ya hay evidencia suficiente de que lo implementado esta mejorando el sistema vivo.
- La observabilidad va por `93.33/100`: esto mide si el tablero operativo realmente esta devolviendo feedback confiable para vigilar la implantacion.

## Next Actions

- Cerrar la brecha de estrategia no atribuida: el libro abierto no debe seguir en untracked.
- Aun no hay muestra cerrada suficiente: usar paper controlado y no declarar mejora de edge todavia.
- Mejorar la entrega a memoria/bitacora: la absorcion real del aprendizaje todavia es irregular.
- Investigar por que alguna recarga Admin API de Grafana sigue siendo fragil aunque los dashboards esten visibles.
