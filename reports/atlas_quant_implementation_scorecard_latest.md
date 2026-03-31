# ATLAS Quant - Trading Implementation Scorecard

- Generated at: `2026-03-31T21:51:39.303057+00:00`
- Process compliance: `97.5/100` (healthy)
- Implementation usefulness: `55.0/100` (watch)

## Metrics

- `process_compliance_score`: `97.5/100` (healthy) - Medir cuanto del marco metodologico ya esta realmente implantado por etapa.
- `artifact_coverage_score`: `100.0/100` (healthy) - Medir si cada avance deja huella en protocolo, informes y archivos reutilizables.
- `memory_persistence_score`: `83.63/100` (workable) - Medir si ATLAS esta reteniendo los cambios en memoria y bitacora.
- `external_benchmark_coverage_score`: `100.0/100` (healthy) - Medir cuantas etapas auditadas estan respaldadas por confrontacion externa seria.
- `observability_feedback_score`: `73.33/100` (workable) - Medir si Grafana, Prometheus y el chequeo operativo estan devolviendo feedback confiable para sostener disciplina.
- `visual_benchmark_feedback_score`: `16.67/100` (critical) - Medir si el benchmark visual externo ya fue traducido a criterios reutilizables, reportes y controles reales de entrada.
- `options_strategy_governance_feedback_score`: `16.67/100` (critical) - Medir si el benchmark externo de estructuras con opciones ya fue traducido a taxonomia, constraints y reglas reutilizables.
- `test_guardrail_score`: `100.0/100` (healthy) - Medir cuanta proteccion automatizada existe sobre lo ya implantado.
- `implementation_usefulness_score`: `55.0/100` (watch) - Medir si ya existe evidencia operativa suficiente para afirmar que los cambios sirven de algo.
- `signal_ic_quality_score`: `0.0/100` (insufficient_data) - Medir la calidad predictiva real de las señales del scanner via IC (Information Coefficient). IC > 0.05 meaningful, IC > 0.10 fuerte (Grinold-Kahn). Score=0 mientras no haya muestra suficiente; crece con evidencia real.

## Supporting Indicators

- `brain_delivery_ratio_pct`: `98.05`
- `attributed_open_positions_pct`: `100.0`
- `open_untracked_ratio_pct`: `0.0`
- `recent_unattributed_count`: `0`
- `evidence_sufficiency_score`: `100.0`
- `post_mortem_coverage_pct`: `100.0`
- `recent_attribution_pct`: `100.0`
- `external_learning_translation_score`: `100.0`
- `observability_feedback_score`: `73.33`
- `visual_benchmark_feedback_score`: `16.67`
- `visual_benchmark_source_count`: `0`
- `visual_benchmark_translation_pct`: `0.0`
- `options_strategy_governance_feedback_score`: `16.67`
- `options_governance_source_count`: `0`
- `options_governance_translation_pct`: `0.0`
- `grafana_reload_resilience_pct`: `66.67`
- `grafana_alerting_ready_pct`: `100.0`
- `signal_ic_quality_score`: `0.0` (IC=None, status=insufficient_data, n=0)

## Interpretation

- El proceso va por `97.5/100`: esto mide implantacion metodologica, no edge de mercado.
- La utilidad va por `55.0/100`: esto mide si ya hay evidencia suficiente de que lo implementado esta mejorando el sistema vivo.
- La observabilidad va por `73.33/100`: esto mide si el tablero operativo realmente esta devolviendo feedback confiable para vigilar la implantacion.
- El benchmark visual va por `16.67/100`: esto mide si la investigacion visual externa ya fue aterrizada a controles reutilizables en ATLAS.
- La gobernanza de opciones va por `16.67/100`: esto mide si el benchmark externo de estructuras con opciones ya fue traducido a familias, reglas y restricciones reutilizables.
- La calidad de señal IC va por `0.0/100`: esto mide el poder predictivo real del scanner (IC Grinold-Kahn). 0=sin datos, 100=IC>=0.10 significativo.

## Next Actions

- Subir la confiabilidad de observabilidad: Grafana/Prometheus aun no devuelven feedback lo bastante robusto.
- Investigar por que alguna recarga Admin API de Grafana sigue siendo fragil aunque los dashboards esten visibles.
- Activar ICSignalTracker: el loop debe llamar tracker.record_signal() al evaluar candidatos.
- La calidad paper sigue debil: no declarar utilidad alta mientras profit_factor/expectancy sigan en zona pobre.
- El paper book aun no tiene profit factor >= 1.0: revisar reconciliacion, sizing y calidad de señal antes de escalar.
- La expectancy paper sigue no positiva: exigir mejora real de payoff antes de considerar que la implantacion ya sirve.
