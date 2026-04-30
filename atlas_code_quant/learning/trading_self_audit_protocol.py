from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from atlas_code_quant.operations.brain_bridge import QuantBrainBridge


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


TRADING_ANALYSIS_SCHEMA: list[dict[str, Any]] = [
    {
        "step": "detect_internal_degradation",
        "goal": "Detectar si la etapa esta deteriorando calidad, riesgo o consistencia operativa.",
        "questions": [
            "que metricas se estan alejando del comportamiento sano esperado",
            "que sintomas se repiten en resultados, logs, journal o dashboards",
            "si el problema es puntual, sistemico o de regimen",
        ],
        "outputs": [
            "alerta de degradacion",
            "lista inicial de sintomas",
        ],
    },
    {
        "step": "collect_internal_evidence",
        "goal": "Levantar evidencia local suficiente antes de inferir una causa raiz.",
        "questions": [
            "que variables, decisiones y resultados reales produce hoy la etapa",
            "que campos faltan para atribuir resultado, calidad o riesgo",
            "que diferencias hay entre lo planeado, lo emitido y lo ejecutado",
        ],
        "outputs": [
            "evidencia interna versionada",
            "huecos de trazabilidad",
        ],
    },
    {
        "step": "research_external_benchmarks",
        "goal": "Buscar referencias serias en red para contrastar criterios y practicas reconocidas.",
        "questions": [
            "que metricas usan sistemas o literatura reconocida para esta etapa",
            "que filtros, guardrails o regimenes suelen aplicar",
            "que errores tipicos se observan cuando esta etapa falla",
        ],
        "outputs": [
            "benchmark externo",
            "lista de practicas comparables",
        ],
    },
    {
        "step": "compare_internal_vs_external",
        "goal": "Comparar lo que hace ATLAS contra el benchmark externo y contra el resultado obtenido.",
        "questions": [
            "que hace ATLAS de forma consistente con el benchmark",
            "que omite ATLAS o usa con peso incorrecto",
            "que metricas sobran, faltan o estan mal conectadas al resultado",
        ],
        "outputs": [
            "gap analysis",
            "mapa de desalineaciones",
        ],
    },
    {
        "step": "diagnose_root_cause",
        "goal": "Separar causa raiz de sintomas superficiales.",
        "questions": [
            "el problema nace en criterio, datos, gobernanza, ejecucion o mezcla de capas",
            "que parte del dano explica mas perdida o mas ruido",
            "que cambio pequeno moveria mas el resultado",
        ],
        "outputs": [
            "causa raiz priorizada",
            "suposiciones explicitadas",
        ],
    },
    {
        "step": "design_corrective_action",
        "goal": "Definir correcciones medibles, reversibles y con criterio de exito claro.",
        "questions": [
            "que pesos, vetos, validaciones o limites deben cambiar",
            "que debe bloquearse de inmediato y que puede ir a observacion",
            "como se medira si la correccion mejora o empeora la etapa",
        ],
        "outputs": [
            "plan de remediacion",
            "criterios before_after",
        ],
    },
    {
        "step": "monitor_after_change",
        "goal": "Seguir el comportamiento despues del cambio para evitar conclusiones prematuras.",
        "questions": [
            "mejora la calidad esperada sin generar efectos secundarios peores",
            "el cambio aguanta una sesion seria o solo una prueba corta",
            "que metricas deben permanecer en verde para sostener el ajuste",
        ],
        "outputs": [
            "seguimiento post-cambio",
            "riesgos residuales",
        ],
    },
    {
        "step": "persist_learning",
        "goal": "Guardar la conclusion y su evidencia para aprendizaje acumulativo.",
        "questions": [
            "que debe registrarse en memoria, bitacora e informe",
            "que aprendizaje es reusable por otras etapas",
            "que condiciones obligan a revisar o revertir despues",
        ],
        "outputs": [
            "nota persistida",
            "politica candidata",
        ],
    },
    {
        "step": "promote_or_revert_policy",
        "goal": "Promover solo lo que resiste evidencia y revertir lo fragil.",
        "questions": [
            "la mejora es sostenida y atribuible",
            "hay criterio de no-go o rollback si el dano reaparece",
            "queda el siguiente foco listo para auditar con el mismo metodo",
        ],
        "outputs": [
            "decision promote_hold_revert",
            "siguiente foco de auditoria",
        ],
    },
    {
        "title": "Credit vs. Debit Spreads: Let Volatility Guide You",
        "url": "https://www.schwab.com/learn/story/credit-vs-debit-spreads-let-volatility-guide-you",
        "domain": "schwab.com",
        "source_type": "broker_guide",
        "used_for": [
            "options_strategy_governance",
        ],
    },
    {
        "title": "Guide to Collars Transcript",
        "url": "https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/learning-center/Transcript_Guide%20to%20collars_v2.pdf",
        "domain": "fidelity.com",
        "source_type": "broker_guide",
        "used_for": [
            "options_strategy_governance",
        ],
    },
    {
        "title": "The OCC Options Strategies",
        "url": "https://www.theocc.com/strategies",
        "domain": "theocc.com",
        "source_type": "industry_guide",
        "used_for": [
            "options_strategy_governance",
        ],
    },
]


TRADING_PROCESS_GUARDRAILS: list[str] = [
    "no promover cambios a live solo por una prueba puntual",
    "no aceptar mejoras sin trazabilidad y comparativa before_after",
    "no confundir indicador tactico con criterio estructural",
    "no cerrar una etapa sin dejar memoria persistida y criterios de seguimiento",
    "no extender el algoritmo a la siguiente etapa si la anterior sigue ciega",
]


EXTERNAL_BENCHMARK_SCAN_FLOW: list[dict[str, Any]] = [
    {
        "step": "detect_stage_question",
        "goal": "Identificar que etapa del proceso de trading necesita contraste externo y con que pregunta concreta.",
        "outputs": [
            "stage_target",
            "benchmark_question",
        ],
    },
    {
        "step": "select_source_family",
        "goal": "Elegir primero literatura y documentos regulatorios o institucionales antes que opinion informal.",
        "source_priority": [
            "papers_academicos",
            "regulatory_guidance",
            "broker_execution_quality_docs",
            "tool_or_feed_docs",
        ],
        "outputs": [
            "source_family",
            "candidate_urls",
        ],
    },
    {
        "step": "scan_sources",
        "goal": "Escanear cada pagina buscando metricas, guardrails, fallos tipicos y condiciones de uso.",
        "extract_fields": [
            "metricas_clave",
            "benchmarks",
            "failure_modes",
            "guardrails",
            "limits_or_disclaimers",
        ],
        "outputs": [
            "source_notes",
            "quoted_or_paraphrased_evidence",
        ],
    },
    {
        "step": "compare_against_atlas",
        "goal": "Comparar lo extraido con la implementacion real de ATLAS y con el resultado observado.",
        "outputs": [
            "atlas_matches",
            "atlas_gaps",
            "wrong_weights_or_missing_gates",
        ],
    },
    {
        "step": "translate_to_action",
        "goal": "Convertir la comparacion en criterios medibles, reversibles y trazables dentro del algoritmo.",
        "outputs": [
            "new_metrics",
            "new_gates",
            "new_reports_or_tests",
        ],
    },
    {
        "step": "persist_external_learning",
        "goal": "Guardar la experiencia externa util para que ATLAS la vuelva a usar por etapa.",
        "outputs": [
            "source_registry_update",
            "memory_note",
            "follow_up_stage",
        ],
    },
]


EXTERNAL_BENCHMARK_SOURCE_REGISTRY: list[dict[str, Any]] = [
    {
        "title": "Time Series Momentum",
        "url": "https://elmwealth.com/wp-content/uploads/2017/06/timeseriesmomentum.pdf",
        "domain": "elmwealth.com",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "trajectory_persistence",
        ],
    },
    {
        "title": "Momentum: Evidence and Insights 30 Years Late",
        "url": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=4602426",
        "domain": "papers.ssrn.com",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "momentum_benchmark",
        ],
    },
    {
        "title": "Residual Momentum",
        "url": "https://repub.eur.nl/pub/22252/ResidualMomentum-2011.pdf",
        "domain": "repub.eur.nl",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "residual_momentum",
        ],
    },
    {
        "title": "Momentum Crashes",
        "url": "https://www.nber.org/papers/w20439.pdf",
        "domain": "nber.org",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "crash_risk",
        ],
    },
    {
        "title": "Volatility Managed Portfolios",
        "url": "https://conference.nber.org/confer/2016/LTAMs16/Moreira_Muir.pdf",
        "domain": "conference.nber.org",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "volatility_regime",
        ],
    },
    {
        "title": "Do Industries Explain Momentum?",
        "url": "https://spinup-000d1a-wp-offload-media.s3.amazonaws.com/faculty/wp-content/uploads/sites/3/2019/09/DoIndustriesExplainMomentum.pdf",
        "domain": "spinup-000d1a-wp-offload-media.s3.amazonaws.com",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "sector_industry_confirmation",
        ],
    },
    {
        "title": "Short-Term Return Reversal: The Long and the Short of It",
        "url": "https://econen.sufe.edu.cn/_upload/article/files/d3/34/c6aa75294353b29e32728df8b04e/01c74230-970a-454a-8483-265efa7c6ba1.pdf",
        "domain": "econen.sufe.edu.cn",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "entry_validation",
            "short_horizon_noise",
        ],
    },
    {
        "title": "Illiquidity and Stock Returns: Cross-Section and Time-Series Effects",
        "url": "https://w4.stern.nyu.edu/finance/docs/WP/2000/pdf/wpa00041.pdf",
        "domain": "w4.stern.nyu.edu",
        "source_type": "paper",
        "used_for": [
            "scanner_selection",
            "entry_validation",
            "liquidity_filters",
        ],
    },
    {
        "title": "Legal Information — yfinance",
        "url": "https://ericpien.github.io/yfinance/getting_started/legal.html",
        "domain": "ericpien.github.io",
        "source_type": "documentation",
        "used_for": [
            "scanner_selection",
            "data_feed_limitations",
        ],
    },
    {
        "title": "FINRA Regulatory Notice 21-23",
        "url": "https://www.finra.org/sites/default/files/2021-06/Regulatory-Notice-21-23.pdf",
        "domain": "finra.org",
        "source_type": "regulatory",
        "used_for": [
            "entry_validation",
            "execution_quality",
            "best_execution_factors",
        ],
    },
    {
        "title": "Transaction Cost Analysis and Implementation Shortfall",
        "url": "https://pages.stern.nyu.edu/~jhasbrou/STPP/drafts/STPPms13b.pdf",
        "domain": "pages.stern.nyu.edu",
        "source_type": "reference_text",
        "used_for": [
            "entry_validation",
            "execution_quality",
            "arrival_price_benchmark",
        ],
    },
    {
        "title": "Commitment to Execution Quality",
        "url": "https://www.fidelity.com/trading/execution-quality/overview",
        "domain": "fidelity.com",
        "source_type": "broker_execution_doc",
        "used_for": [
            "entry_validation",
            "execution_quality",
            "effective_spread",
            "price_improvement",
        ],
    },
    {
        "title": "Chart Patterns Transcript",
        "url": "https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/learning-center/Transcript_Chart%20patterns_v2.pdf",
        "domain": "fidelity.com",
        "source_type": "broker_education_doc",
        "used_for": [
            "entry_validation",
            "visual_entry_optimization",
            "breakout_confirmation",
            "pullback_confirmation",
        ],
    },
    {
        "title": "How to Read Stock Charts and Trading Patterns",
        "url": "https://www.schwab.com/learn/story/how-to-read-stock-charts-and-trading-patterns",
        "domain": "schwab.com",
        "source_type": "broker_education_doc",
        "used_for": [
            "entry_validation",
            "visual_entry_optimization",
            "timeframe_structure_confirmation",
        ],
    },
    {
        "title": "Technical Indicators: 3 Trading Traps to Avoid",
        "url": "https://www.schwab.com/learn/story/technical-indicators-3-trading-traps-to-avoid",
        "domain": "schwab.com",
        "source_type": "broker_education_doc",
        "used_for": [
            "entry_validation",
            "visual_entry_optimization",
            "false_breakout_avoidance",
        ],
    },
    {
        "title": "Using the Volume Profile Indicator",
        "url": "https://workplace.schwab.com/story/using-volume-profile-indicator",
        "domain": "schwab.com",
        "source_type": "broker_education_doc",
        "used_for": [
            "entry_validation",
            "visual_entry_optimization",
            "price_acceptance_rejection",
        ],
    },
    {
        "title": "Credit vs. Debit Spreads: Let Volatility Guide You",
        "url": "https://www.schwab.com/learn/story/credit-vs-debit-spreads-let-volatility-guide-you",
        "domain": "schwab.com",
        "source_type": "broker_guide",
        "used_for": [
            "options_strategy_governance",
        ],
    },
    {
        "title": "Guide to Collars Transcript",
        "url": "https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/learning-center/Transcript_Guide%20to%20collars_v2.pdf",
        "domain": "fidelity.com",
        "source_type": "broker_guide",
        "used_for": [
            "options_strategy_governance",
        ],
    },
    {
        "title": "The OCC Options Strategies",
        "url": "https://www.theocc.com/strategies",
        "domain": "theocc.com",
        "source_type": "industry_guide",
        "used_for": [
            "options_strategy_governance",
        ],
    },
    {
        "title": "Trend Following with Managed Futures: Historical Perspectives",
        "url": "https://www.cmegroup.com/content/dam/cmegroup/education/files/trend-following-with-managed-futures-historical-perspectives.pdf",
        "domain": "cmegroup.com",
        "source_type": "education_reference",
        "used_for": [
            "scanner_selection",
            "entry_validation",
            "breakout_discipline",
            "trend_context",
        ],
    },
    {
        "title": "Proper Position Size",
        "url": "https://www.cmegroup.com/education/courses/trade-and-risk-management/the-2-percent-rule.html",
        "domain": "cmegroup.com",
        "source_type": "education_reference",
        "used_for": [
            "position_management",
            "position_sizing_discipline",
            "risk_per_trade",
        ],
    },
    {
        "title": "Beginners' Guide to Asset Allocation, Diversification, and Rebalancing",
        "url": "https://www.investor.gov/sites/investorgov/files/2019-02/Beginners-Guide-to-Asset-Allocation.pdf",
        "domain": "investor.gov",
        "source_type": "regulatory_education",
        "used_for": [
            "position_management",
            "portfolio_concentration",
            "diversification_discipline",
        ],
    },
    {
        "title": "Things to Know About Order Types",
        "url": "https://www.fidelity.com/bin-public/060_www_fidelity_com/documents/active-trader/things-to-know.pdf",
        "domain": "fidelity.com",
        "source_type": "broker_execution_doc",
        "used_for": [
            "exit_governance",
            "stop_loss_orders",
            "trailing_stop_design",
        ],
    },
    {
        "title": "Trade Log",
        "url": "https://www.cmegroup.com/education/courses/trade-and-risk-management/trade-log.html",
        "domain": "cmegroup.com",
        "source_type": "education_reference",
        "used_for": [
            "post_trade_learning",
            "trade_review_loop",
            "journaling_discipline",
        ],
    },
    # ── Nuevas fuentes incorporadas en auditoría 2026-03-28 ───────────────────
    {
        "title": "Active Portfolio Management — Grinold & Kahn (2000)",
        "url": "https://www.mhprofessional.com/active-portfolio-management-9780070248823-usa",
        "domain": "mhprofessional.com",
        "source_type": "reference_book",
        "key_benchmarks": {
            "IC_threshold_meaningful": 0.05,
            "IC_threshold_strong": 0.10,
            "t_stat_min": 2.0,
            "IR_viable": 0.5,
            "formula": "IR = IC × sqrt(Breadth)",
        },
        "used_for": [
            "scanner_selection",
            "signal_ic_quality",
            "information_coefficient_tracking",
        ],
    },
    {
        "title": "The Implementation Shortfall — Perold (1988)",
        "url": "https://pages.stern.nyu.edu/~jhasbrou/STPP/drafts/STPPms13b.pdf",
        "domain": "pages.stern.nyu.edu",
        "source_type": "paper",
        "key_benchmarks": {
            "arrival_price_slippage_liquid_bps": 5,
            "arrival_price_slippage_midcap_bps": 15,
            "adverse_selection_threshold_pct": 50,
            "IS_components": ["delay_cost", "market_impact", "timing_risk", "opportunity_cost"],
        },
        "used_for": [
            "execution_quality",
            "entry_validation",
            "implementation_shortfall_decomposition",
        ],
    },
    {
        "title": "Trade Your Way to Financial Freedom — Van Tharp (2006)",
        "url": "https://www.mhprofessional.com/9780071474467-usa-trade-your-way-to-financial-freedom",
        "domain": "mhprofessional.com",
        "source_type": "reference_book",
        "key_benchmarks": {
            "expectancy_r_viable": 0.20,
            "expectancy_r_excellent": 0.50,
            "atr_trailing_stop_range": [2.0, 3.0],
            "r_multiple_min_payoff": 2.0,
        },
        "used_for": [
            "position_management",
            "exit_governance",
            "r_multiple_system",
            "stop_loss_calibration",
        ],
    },
    {
        "title": "When Do Stop-Loss Rules Stop Losses? — Kaminski & Lo (2014)",
        "url": "https://www.sciencedirect.com/science/article/pii/S1386418113000366",
        "domain": "sciencedirect.com",
        "source_type": "paper",
        "key_benchmarks": {
            "stop_adds_value_when": "combined with full exit-and-reentry protocol",
            "time_stop_recommendation": "close positions not at +0.5R within 50% of expected holding period",
            "mfe_trailing_activation_r": 1.0,
        },
        "used_for": [
            "exit_governance",
            "position_management",
            "time_stop_discipline",
        ],
    },
    {
        "title": "Determinants of Portfolio Performance — Brinson, Hood & Beebower (1986)",
        "url": "https://www.cfainstitute.org/en/membership/professional-development/refresher-readings/portfolio-performance-evaluation",
        "domain": "cfainstitute.org",
        "source_type": "paper",
        "key_benchmarks": {
            "attribution_components": ["allocation_effect", "selection_effect", "interaction_effect"],
            "min_sample_per_stratum": 30,
            "stability_criterion_ratio": 0.70,
        },
        "used_for": [
            "post_trade_learning",
            "bhb_attribution_decomposition",
            "strategy_performance_attribution",
        ],
    },
    {
        "title": "Advances in Financial Machine Learning — Lopez de Prado (2018), Cap. 14",
        "url": "https://www.wiley.com/en-us/Advances+in+Financial+Machine+Learning-p-9781119482086",
        "domain": "wiley.com",
        "source_type": "reference_book",
        "key_benchmarks": {
            "min_trades_live_significance": 300,
            "min_calendar_months": 3,
            "min_profit_factor_paper": 1.5,
            "max_drawdown_pct": 15.0,
            "min_calmar_ratio": 1.5,
            "min_sharpe_dsr_adjusted": 1.0,
            "stability_score_min": 0.70,
        },
        "used_for": [
            "post_trade_learning",
            "live_transition_criteria",
            "deflated_sharpe_adjustment",
            "combinatorial_cross_validation",
        ],
    },
]


IMPLEMENTATION_SCORECARD_METRICS: list[dict[str, str]] = [
    {
        "name": "process_compliance_score",
        "goal": "Medir cuanto del marco metodologico ya esta realmente implantado por etapa.",
    },
    {
        "name": "artifact_coverage_score",
        "goal": "Medir si cada avance deja huella en protocolo, informes y archivos reutilizables.",
    },
    {
        "name": "memory_persistence_score",
        "goal": "Medir si ATLAS esta reteniendo los cambios en memoria y bitacora.",
    },
    {
        "name": "external_benchmark_coverage_score",
        "goal": "Medir cuantas etapas auditadas estan respaldadas por confrontacion externa seria.",
    },
    {
        "name": "observability_feedback_score",
        "goal": "Medir si Grafana, Prometheus y el chequeo operativo estan devolviendo feedback confiable para sostener disciplina.",
    },
    {
        "name": "visual_benchmark_feedback_score",
        "goal": "Medir si el benchmark visual externo ya fue traducido a criterios reutilizables, reportes y controles reales de entrada.",
    },
    {
        "name": "options_strategy_governance_feedback_score",
        "goal": "Medir si el benchmark externo de estructuras con opciones ya fue traducido a taxonomia, constraints y reglas reutilizables.",
    },
    {
        "name": "hybrid_order_flow_feedback_score",
        "goal": (
            "Medir si el order flow hibrido (microestructura intradia + superficie de opciones) "
            "ya fue traducido a proveedor reusable, telemetria degradable, status operativo y tests."
        ),
    },
    {
        "name": "test_guardrail_score",
        "goal": "Medir cuanta proteccion automatizada existe sobre lo ya implantado.",
    },
    {
        "name": "implementation_usefulness_score",
        "goal": "Medir si ya existe evidencia operativa suficiente para afirmar que los cambios sirven de algo.",
    },
    {
        "name": "signal_ic_quality_score",
        "goal": (
            "Medir la calidad predictiva real de las señales del scanner via IC (Information Coefficient). "
            "IC > 0.05 meaningful, IC > 0.10 fuerte (Grinold-Kahn). "
            "Score=0 mientras no haya muestra suficiente; crece con evidencia real."
        ),
    },
]


TRADING_STAGE_BLUEPRINTS: dict[str, dict[str, Any]] = {
    "scanner_selection": {
        "status": "baseline_hardened",
        "goal": "Seleccionar activos y direcciones con trayectoria mas robusta y expectativa mas sana.",
        "core_question": "Que criterios de seleccion anticipan mejor la trayectoria futura con payoff sano y muestra defendible.",
        "internal_evidence": [
            "ranking del scanner",
            "metricas por metodo y timeframe",
            "rechazos, aceptaciones y score breakdown",
            "correlacion posterior con pnl abierto y cerrado",
        ],
        "external_benchmark_focus": [
            "momentum multi-horizonte",
            "trend following",
            "residual momentum",
            "volatility managed selection",
            "crash-risk control",
        ],
        "failure_modes": [
            "premiar hit rate local por encima de expectativa",
            "mezclar intradia y swing sin separacion",
            "no filtrar regimen de volatilidad ni contexto sectorial",
        ],
        "corrective_actions": [
            "recalibrar selection_score",
            "vetar profit factor y muestra debiles",
            "separar motores intradia y swing",
            "añadir filtros de regimen y confirmacion estructural",
        ],
        "success_metrics": [
            "mejor expectativa por candidato",
            "menor burst de entradas fragiles",
            "menor dano en longs",
        ],
    },
    "entry_validation": {
        "status": "baseline_hardened",
        "goal": "Validar que la entrada elegida siga siendo valida al momento del ticket.",
        "core_question": "La tesis seleccionada sigue viva en el momento exacto de entrada o ya se deterioro.",
        "internal_evidence": [
            "precio de decision vs precio de ticket",
            "spread, liquidez y slippage esperado",
            "estructura de precio al disparo",
            "motivo de bloqueo o aprobacion",
        ],
        "external_benchmark_focus": [
            "microestructura",
            "confirmacion de breakout o pullback",
            "volatility adjusted entries",
            "liquidity filters",
        ],
        "failure_modes": [
            "entrar tarde sobre extension agotada",
            "ignorar spread, liquidez o gap de informacion",
            "perseguir un setup ya roto",
        ],
        "corrective_actions": [
            "gates de deterioro de setup",
            "umbrales de spread y liquidez",
            "confirmacion final de estructura antes del submit",
        ],
        "success_metrics": [
            "menor slippage de entrada",
            "menor entrada sobre extremos agotados",
            "mayor conversion de scan sano a ticket sano",
        ],
    },
    "execution_quality": {
        "status": "baseline_hardened",
        "goal": "Asegurar que la orden enviada coincide con la tesis y no introduce deterioro evitable.",
        "core_question": "La capa de ejecucion esta preservando la tesis o la esta degradando con errores operativos.",
        "internal_evidence": [
            "decision planificada vs orden emitida",
            "duplicados, reentradas y stacking",
            "latencia, rechazos y reconciliacion",
            "estado del broker y del canonical snapshot",
        ],
        "external_benchmark_focus": [
            "order quality controls",
            "duplicate prevention",
            "pre-trade risk checks",
            "reconciliation discipline",
        ],
        "failure_modes": [
            "duplicados o stacking no autorizado",
            "desalineacion entre orden y tesis",
            "falsa sensacion de sincronizacion",
        ],
        "corrective_actions": [
            "guards pre-submit",
            "reconciliacion obligatoria",
            "validacion de metadata y trazabilidad por estrategia",
        ],
        "success_metrics": [
            "cero duplicados no autorizados",
            "cero trades untracked",
            "sync_status sano sostenido",
        ],
    },
    "position_management": {
        "status": "baseline_hardened",
        "goal": "Gestionar el riesgo una vez abierta la posicion.",
        "core_question": "La posicion viva sigue comportandose como la tesis esperaba o ya esta degradandose.",
        "internal_evidence": [
            "mae, mfe y hold time",
            "cambios de volatilidad y correlacion",
            "exposicion por simbolo, sector y libro",
            "cambios de score o tesis durante la vida del trade",
        ],
        "external_benchmark_focus": [
            "position sizing discipline",
            "volatility scaling",
            "portfolio heat",
            "drawdown containment",
        ],
        "failure_modes": [
            "dejar crecer perdedores",
            "no reducir riesgo cuando el setup se invalida",
            "sobreconcentracion del libro",
        ],
        "corrective_actions": [
            "limites de exposicion",
            "reglas de reduccion y invalidacion",
            "seguimiento de excursion favorable y adversa",
        ],
        "success_metrics": [
            "menor perdida media",
            "menor concentracion toxica",
            "mejor relacion riesgo-retorno viva",
        ],
    },
    "exit_governance": {
        "status": "baseline_hardened",
        "goal": "Salir por criterio, no por ruido ni improvisacion.",
        "core_question": "La salida se ejecuta por una regla sana y coherente con la tesis original.",
        "internal_evidence": [
            "motivo de salida",
            "pnl realizado por tipo de salida",
            "hold time real vs planeado",
            "invalidez, target o timeout",
        ],
        "external_benchmark_focus": [
            "time stop discipline",
            "invalidation exits",
            "profit target governance",
            "trailing stop design",
        ],
        "failure_modes": [
            "cortar demasiado pronto ganadores",
            "dejar correr demasiado perdedores",
            "salir sin relacion con la tesis",
        ],
        "corrective_actions": [
            "clasificacion estructurada de exits",
            "reglas por invalidez, objetivo y tiempo",
            "comparativa de eficiencia de salida",
        ],
        "success_metrics": [
            "mejor payoff realizado",
            "menor fuga de ganadores",
            "menor tiempo muerto en trades rotos",
        ],
    },
    "post_trade_learning": {
        "status": "active_focus",
        "goal": "Convertir lo ocurrido en conocimiento util y reusable.",
        "core_question": "El sistema esta aprendiendo de forma acumulativa o solo acumulando eventos sin criterio.",
        "internal_evidence": [
            "journal y canonico enriquecidos",
            "memoria y bitacora con contexto suficiente",
            "politicas promovidas, descartadas o revertidas",
            "comparativa expectativa vs resultado",
        ],
        "external_benchmark_focus": [
            "trading review loops",
            "model governance",
            "evidence-based adaptation",
            "policy promotion and rollback",
        ],
        "failure_modes": [
            "persistir eventos sin aprendizaje",
            "cambiar reglas sin evidencia acumulada",
            "no cerrar el loop between result and policy",
        ],
        "corrective_actions": [
            "plantilla unica de aprendizaje",
            "reporte before_after",
            "criterio formal de promote_hold_revert",
        ],
        "success_metrics": [
            "memoria util y consultable",
            "politicas con evidencia",
            "aprendizaje reusable entre etapas",
        ],
    },
}


TRADING_SELF_AUDIT_LIFECYCLE: list[dict[str, Any]] = [
    {
        "stage": "scanner_selection",
        "status": TRADING_STAGE_BLUEPRINTS["scanner_selection"]["status"],
        "goal": TRADING_STAGE_BLUEPRINTS["scanner_selection"]["goal"],
        "checks": [
            "verificar que selection_score no premie solo hit rate local",
            "medir expectancy, profit_factor y sample_size por metodo y timeframe",
            "comparar impulso temporal, fuerza relativa y contexto sectorial",
            "separar ruido intradia de trayectoria swing",
            "validar el feed de datos para el horizonte usado",
        ],
        "outputs": [
            "ranking corregido",
            "metricas faltantes",
            "veto de criterios fragiles",
        ],
        "analysis_schema_ref": "TRADING_ANALYSIS_SCHEMA",
    },
    {
        "stage": "entry_validation",
        "status": TRADING_STAGE_BLUEPRINTS["entry_validation"]["status"],
        "goal": TRADING_STAGE_BLUEPRINTS["entry_validation"]["goal"],
        "checks": [
            "confirmar que el setup no se haya degradado desde el scan",
            "confirmar estructura de precio, liquidez y spread",
            "verificar que la entrada respete el contexto de tendencia y volatilidad",
        ],
        "outputs": [
            "entrada aprobada o bloqueada",
            "motivo estructurado de entrada",
        ],
        "analysis_schema_ref": "TRADING_ANALYSIS_SCHEMA",
    },
    {
        "stage": "execution_quality",
        "status": TRADING_STAGE_BLUEPRINTS["execution_quality"]["status"],
        "goal": TRADING_STAGE_BLUEPRINTS["execution_quality"]["goal"],
        "checks": [
            "comparar decision planificada vs orden real",
            "detectar duplicados, reentradas y stacking no autorizados",
            "medir slippage, latencia y reconciliacion",
        ],
        "outputs": [
            "diagnostico de calidad de ejecucion",
            "alertas de desvio operativo",
        ],
        "analysis_schema_ref": "TRADING_ANALYSIS_SCHEMA",
    },
    {
        "stage": "position_management",
        "status": TRADING_STAGE_BLUEPRINTS["position_management"]["status"],
        "goal": TRADING_STAGE_BLUEPRINTS["position_management"]["goal"],
        "checks": [
            "vigilar deterioro del setup frente al plan",
            "medir excursion favorable y adversa",
            "comparar exposicion viva con limites del libro",
        ],
        "outputs": [
            "ajustes de riesgo",
            "decision de mantener, reducir o cerrar",
        ],
        "analysis_schema_ref": "TRADING_ANALYSIS_SCHEMA",
    },
    {
        "stage": "exit_governance",
        "status": TRADING_STAGE_BLUEPRINTS["exit_governance"]["status"],
        "goal": TRADING_STAGE_BLUEPRINTS["exit_governance"]["goal"],
        "checks": [
            "medir si el motivo de salida coincide con la tesis original",
            "comparar salidas por objetivo, invalidez o tiempo",
            "detectar si los peores trades se dejan correr demasiado",
        ],
        "outputs": [
            "clasificacion de salida",
            "causa raiz del cierre",
        ],
        "analysis_schema_ref": "TRADING_ANALYSIS_SCHEMA",
    },
    {
        "stage": "post_trade_learning",
        "status": TRADING_STAGE_BLUEPRINTS["post_trade_learning"]["status"],
        "goal": TRADING_STAGE_BLUEPRINTS["post_trade_learning"]["goal"],
        "checks": [
            "registrar resultado, contexto y causa raiz",
            "comparar expectativa prometida vs resultado real",
            "actualizar memoria, bitacora y politicas si la evidencia se sostiene",
        ],
        "outputs": [
            "nota de aprendizaje persistida",
            "politica promovida o descartada",
        ],
        "analysis_schema_ref": "TRADING_ANALYSIS_SCHEMA",
    },
]


SCANNER_SELECTION_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "scanner_selection",
    "key_findings": [
        "El scanner actual esta inspirado en momentum y trend-following validos, pero el ranking esta sesgado hacia metricas locales y recientes.",
        "selection_score sobrepremia local_win_rate y no incorpora con suficiente peso expectancy, profit_factor y sample_size.",
        "predicted_move_pct mide amplitud volatilidad-escalada, no trayectoria futura robusta.",
        "Faltan filtros fuertes de crash regime, residual momentum, contexto sectorial e impulso gestionado por volatilidad.",
        "La mezcla de horizontes intradia y swing dentro del mismo ranking puede sobrealimentar ruido y falsos positivos en longs.",
    ],
    "likely_failures": [
        "optimizar acierto local en vez de expectativa",
        "no penalizar adecuadamente asimetria negativa de payoff",
        "confundir amplitud esperada con direccion persistente",
        "no separar suficientemente regimen intradia y regimen swing",
    ],
    "recommended_metrics": [
        "expectancy_pct por metodo y timeframe",
        "profit_factor local penalizado por muestra baja",
        "avg_win_over_avg_loss",
        "momentum 3m-6m-12m con control de reversión corta",
        "residual momentum o confirmacion sectorial/industrial",
        "volatility regime y volatility scaling",
        "crash-risk veto para momentum long",
    ],
}


ENTRY_VALIDATION_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "entry_validation",
    "key_findings": [
        "ATLAS ya describia bien la entrada en entry_plan, pero esa capa era mayormente narrativa y no un gate operativo duro.",
        "La validacion actual dependia mas de confirmacion visual, probabilidad y playbook que de costo de ejecucion observable.",
        "Faltaba comparar precio de decision vs precio ejecutable justo antes del ticket.",
        "Faltaba usar spread y deriva adversa como criterio de bloqueo para no perseguir entradas deterioradas.",
    ],
    "likely_failures": [
        "entrar tarde cuando el precio ya consumio demasiado del edge",
        "ignorar spread y friccion aunque la tesis siga pareciendo valida en grafico",
        "tratar la camara como sustituto de microestructura cuando deberia ser una confirmacion adicional",
    ],
    "recommended_metrics": [
        "entry_reference_price",
        "adverse_drift_pct",
        "spread_pct",
        "drift_vs_expected_move_pct",
        "arrival-price benchmark before submit",
        "slippage realizado por familia y timeframe",
    ],
    "implemented_p0": [
        "entry_reference_price y thresholds de drift/spread ahora viajan en OrderRequest y order_seed",
        "OperationCenter valida quote, spread y adverse drift antes de preview/submit en equities",
        "auto-cycle transmite precio de referencia y expected_move desde el scanner",
    ],
}


VISUAL_ENTRY_BENCHMARK_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "visual_entry_optimization",
    "human_best_practice": [
        "contexto claro en timeframe superior antes de mirar el disparo fino",
        "nivel tecnico visible y defendible: soporte, resistencia, breakout o pullback",
        "trigger concreto en vez de intuicion: cierre de confirmacion, rechazo, recuperacion o expansion",
        "invalidacion clara antes del ticket",
    ],
    "automation_translation": [
        "chart plan por timeframe y simbolo",
        "expected visual thesis con sesgo y patrones esperados",
        "evidencia OCR y captura persistida",
        "bloqueo fail-closed cuando la confirmacion visual es insuficiente",
    ],
    "external_benchmark_focus": [
        "breakout confirmation",
        "pullback confirmation",
        "multi-timeframe structure",
        "volume and price acceptance",
        "implementation shortfall versus decision benchmark",
    ],
    "key_findings": [
        "El metodo visual mas robusto no es mirar mas cosas, sino exigir contexto, nivel, trigger e invalidacion en ese orden.",
        "Los humanos competentes esperan confirmacion visual y aceptan entrar algo mas tarde a cambio de reducir trampas de breakout.",
        "Los sistemas automaticos serios ganan cuando traducen esa confirmacion a reglas medibles y la confrontan con benchmark de ejecucion.",
        "ATLAS necesitaba dejar de tratar la camara como una captura decorativa y usarla como contraste operativo de la tesis esperada.",
    ],
    "likely_failures": [
        "abrir grafico sin confirmar que lo visible sigue alineado con el setup",
        "usar OCR solo como evidencia de captura y no como contraste de sesgo o patron",
        "exigir el mismo nivel de validacion visual a una equity simple y a una estructura de opciones definida",
    ],
    "recommended_metrics": [
        "visual_readiness_score_pct",
        "visual_alignment_score_pct",
        "ocr_confidence_pct",
        "chart_symbol_match",
        "chart_timeframe_match",
        "capture_evidence_present",
        "visual_gate_block_rate",
    ],
    "web_feedback_loop": [
        "detectar que setups visuales generan mas bloqueos o mas confirmaciones",
        "buscar benchmark externo especifico para ese patron o error tipico",
        "comparar el criterio humano descrito por la fuente con la implementacion real de ATLAS",
        "traducir la comparacion a thresholds, warnings, tests o scorecard",
        "persistir la fuente y la conclusion en memoria para la siguiente iteracion",
    ],
}


OPTIONS_STRATEGY_GOVERNANCE_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "options_strategy_governance",
    "human_best_practice": [
        "separar primero la tesis: direccional, neutral theta, expansion, contraccion o cobertura",
        "usar nivel de IV y su relacion con HV para decidir entre premium comprado, debit spread o credit spread",
        "mirar term structure antes de usar calendar o diagonal",
        "no usar estrategias de cobertura si no existe contexto real de stock o inventario que proteger",
    ],
    "automation_translation": [
        "normalizar tesis y aliases de estrategia antes de seleccionar estructura",
        "distinguir familias: directional_debit, directional_credit, neutral_theta y time_spread",
        "activar calendar o diagonal solo cuando la term structure lo justifica",
        "registrar preferred_but_unavailable_strategy cuando falte contexto para una cobertura real",
    ],
    "external_benchmark_focus": [
        "iv guides credit versus debit",
        "time spreads need term structure support",
        "hedges require stock context",
        "event risk penalizes short premium",
    ],
    "key_findings": [
        "ATLAS estaba demasiado concentrado en verticales e iron condor, con poca discriminacion entre tesis de volatilidad, tiempo y cobertura.",
        "La eleccion de estrategia con opciones no debe depender solo de direccion e IV; tambien importa la curva temporal de volatilidad y si existe posicion subyacente a proteger.",
        "Calendar y diagonal son estructuras utiles cuando la tesis es de tiempo/vega y la term structure lo respalda.",
        "Covered call o protective put son ideas validas, pero no deben activarse automaticamente si el sistema no confirma contexto de stock real en cartera.",
    ],
    "likely_failures": [
        "vender credito en alta IV con evento cercano solo porque la prima parece atractiva",
        "forzar una vertical cuando la tesis real es de carry temporal o term structure",
        "confundir una necesidad de cobertura con una idea direccional pura",
    ],
    "recommended_metrics": [
        "options_strategy_family",
        "volatility_posture",
        "term_structure_slope",
        "event_penalty_applied",
        "preferred_but_unavailable_strategy",
        "time_spread_gap_dte",
    ],
    "web_feedback_loop": [
        "detectar cuando el selector elige demasiadas verticales frente a tesis de tiempo o cobertura",
        "buscar benchmark externo de estructura adecuada para ese contexto",
        "comparar el criterio externo con la taxonomia real del selector",
        "traducir la comparacion a familia, governance o constraints nuevos",
        "persistir la conclusion para que la siguiente iteracion no vuelva a simplificar de mas la capa de opciones",
    ],
}


EXECUTION_QUALITY_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "execution_quality",
    "key_findings": [
        "ATLAS ya ejecutaba ordenes y guardias P0 evitaban varios errores criticos, pero la calidad de ejecucion seguia embebida en payloads crudos.",
        "Faltaba una lectura estructurada de plan vs request_payload vs broker response.",
        "La etapa no distinguia con claridad entre ejecucion sana, degradada o simplemente respondida.",
        "La ausencia de ruta, mismatches de payload o status dudosos del broker debian quedar explicitados para aprendizaje posterior.",
    ],
    "likely_failures": [
        "asumir que una respuesta del broker implica ejecucion sana",
        "no detectar desalineacion entre orden planificada y orden emitida",
        "no registrar degradacion operativa aunque la orden haya sido enviada",
    ],
    "recommended_metrics": [
        "symbol_match",
        "side_match",
        "quantity_match",
        "preview_match",
        "mode_match",
        "order_id_present",
        "broker_status_ok",
        "slippage_realizado",
        "reconciliation_after_send",
    ],
    "implemented_p0": [
        "OperationCenter ahora emite execution_quality separado del blob de execution",
        "se comparan planned_order, request_payload y broker response",
        "ATLAS ya puede etiquetar una ejecucion como degraded aunque no quede bloqueada retrospectivamente",
    ],
}


POSITION_MANAGEMENT_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "position_management",
    "key_findings": [
        "El journal y el tracker ya contienen datos suficientes para medir deterioro de tesis, hold time y calor del libro, pero esa lectura no estaba aterrizada como bloque operativo.",
        "ATLAS necesitaba una vista viva de adverse excursion, stale losers y concentracion por simbolo o familia para no descubrir el dano demasiado tarde.",
        "La gestion actual del libro carecia de una watchlist estructurada que priorizara posiciones por perdida en R, deriva de tesis y tiempo estancado.",
        "Sin esta capa, el sistema puede abrir y ejecutar mejor, pero aun asi dejar que el libro se deteriore por concentracion o por perdedores lentos.",
    ],
    "likely_failures": [
        "dejar crecer perdedores por falta de limites vivos",
        "no detectar sobreconcentracion por simbolo o familia hasta que el libro ya esta cargado",
        "confundir una posicion abierta con una tesis todavia sana",
    ],
    "recommended_metrics": [
        "open_r_multiple",
        "holding_hours",
        "thesis_drift_pct",
        "symbol_heat_pct",
        "strategy_heat_pct",
        "stale_loser_count",
        "adverse_positions_count",
    ],
    "implemented_p0": [
        "journal y operation/status ahora exponen un snapshot dedicado de position_management",
        "ATLAS ya puede detectar stale losers, tesis deterioradas y exceso de calor por simbolo",
        "se genera una watchlist accionable del libro abierto sin tocar el schema de runtime",
    ],
}


EXIT_GOVERNANCE_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "exit_governance",
    "key_findings": [
        "ATLAS ya puede detectar posiciones deterioradas, pero aun no traducia esa evidencia en una recomendacion estructurada de salida.",
        "Faltaba una capa que diferenciara entre invalidacion, time-stop, proteccion de beneficio y simple observacion.",
        "Sin gobierno de salida, una buena deteccion de deterioro puede quedarse en diagnostico sin accion.",
    ],
    "likely_failures": [
        "dejar trades rotos abiertos por no formalizar el criterio de invalidacion",
        "tomar beneficios o recortes de forma inconsistente",
        "mezclar de-risking de concentracion con una salida completa por tesis rota",
    ],
    "recommended_metrics": [
        "exit_now_count",
        "de_risk_count",
        "take_profit_count",
        "time_stop_candidates",
        "thesis_invalidated_candidates",
        "profit_protection_candidates",
    ],
    "implemented_p0": [
        "operation/status ahora expone exit_governance separado de position_management",
        "ATLAS ya clasifica recomendaciones en exit_now, de_risk, take_profit u hold",
        "las recomendaciones quedan trazadas con reason y urgency en vez de depender de lectura manual",
    ],
}


POST_TRADE_LEARNING_DEEP_DIVE: dict[str, Any] = {
    "current_focus": "post_trade_learning",
    "key_findings": [
        "ATLAS ya dispone de trades cerrados, post_mortem parciales y sesgos adaptativos, pero faltaba una lectura compacta que convirtiera cierre en politica candidata.",
        "El aprendizaje posterior al trade no debe limitarse a persistir eventos; debe resumir cobertura de post-mortem, causas raiz y estrategias que piden revision.",
        "Sin esta capa, el sistema puede cerrar mejor pero aun asi no reutilizar de forma disciplinada lo que ya ocurrio.",
    ],
    "likely_failures": [
        "guardar post-mortem sin traducirlo a accion futura",
        "no detectar estrategias que acumulan dano repetido",
        "confundir almacenamiento historico con aprendizaje operativo",
    ],
    "recommended_metrics": [
        "post_mortem_coverage_pct",
        "root_cause_breakdown",
        "strategy_learning_pnl",
        "policy_candidate_count",
        "closed_trades",
        "learning_reuse_readiness",
    ],
    "implemented_p0": [
        "operation/status ahora expone post_trade_learning con causas raiz y candidatos de politica",
        "ATLAS ya puede resumir que estrategias merecen review_reduce_or_disable segun trades cerrados",
        "la cobertura de post_mortem ya entra como señal visible del aprendizaje real",
    ],
}


def build_stage_audit_blueprint(stage: str) -> dict[str, Any]:
    if stage not in TRADING_STAGE_BLUEPRINTS:
        raise KeyError(f"Unknown trading audit stage: {stage}")
    blueprint = dict(TRADING_STAGE_BLUEPRINTS[stage])
    blueprint["stage"] = stage
    blueprint["analysis_schema"] = TRADING_ANALYSIS_SCHEMA
    blueprint["guardrails"] = TRADING_PROCESS_GUARDRAILS
    return blueprint


def build_trading_stage_map() -> list[dict[str, Any]]:
    return [build_stage_audit_blueprint(stage["stage"]) for stage in TRADING_SELF_AUDIT_LIFECYCLE]


def build_trading_self_audit_note() -> dict[str, Any]:
    return {
        "generated_at": _utcnow_iso(),
        "topic": "atlas_quant_trading_self_audit_protocol",
        "current_focus": "post_trade_learning",
        "summary": (
            "ATLAS debe auto-auditar todo el proceso de trading de forma logica y secuencial. "
            "El scanner, la validacion de entrada, la calidad de ejecucion, la gestion viva de posiciones y la gobernanza de salida ya quedaron en una linea base endurecida y en la fase actual el foco esta en post_trade_learning, "
            "pero el algoritmo queda abierto "
            "para extenderse a entrada, ejecucion, gestion de posicion, salida y aprendizaje post-trade."
        ),
        "analysis_schema": TRADING_ANALYSIS_SCHEMA,
        "process_guardrails": TRADING_PROCESS_GUARDRAILS,
        "implementation_scorecard_metrics": IMPLEMENTATION_SCORECARD_METRICS,
        "external_benchmark_scan_flow": EXTERNAL_BENCHMARK_SCAN_FLOW,
        "external_benchmark_sources": EXTERNAL_BENCHMARK_SOURCE_REGISTRY,
        "lifecycle": TRADING_SELF_AUDIT_LIFECYCLE,
        "stage_map": build_trading_stage_map(),
        "scanner_selection_focus": SCANNER_SELECTION_DEEP_DIVE,
        "entry_validation_focus": ENTRY_VALIDATION_DEEP_DIVE,
        "visual_entry_benchmark_focus": VISUAL_ENTRY_BENCHMARK_DEEP_DIVE,
        "options_strategy_governance_focus": OPTIONS_STRATEGY_GOVERNANCE_DEEP_DIVE,
        "execution_quality_focus": EXECUTION_QUALITY_DEEP_DIVE,
        "position_management_focus": POSITION_MANAGEMENT_DEEP_DIVE,
        "exit_governance_focus": EXIT_GOVERNANCE_DEEP_DIVE,
        "post_trade_learning_focus": POST_TRADE_LEARNING_DEEP_DIVE,
    }


def _merge_protocol_focus(default_focus: dict[str, Any], payload_focus: Any) -> dict[str, Any]:
    merged = deepcopy(default_focus)
    if not isinstance(payload_focus, dict):
        return merged
    for key, value in payload_focus.items():
        if value in (None, "", [], {}):
            continue
        merged[key] = deepcopy(value)
    return merged


def _merge_protocol_sources(default_sources: list[dict[str, Any]], payload_sources: Any) -> list[dict[str, Any]]:
    merged: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    payload_list = payload_sources if isinstance(payload_sources, list) else []

    def _register(source: Any) -> None:
        if not isinstance(source, dict):
            return
        title = str(source.get("title") or "").strip().lower()
        url = str(source.get("url") or "").strip().lower()
        if title or url:
            identity = (title, url)
        else:
            identity = ("__source__", json.dumps(source, sort_keys=True, ensure_ascii=True))
        if identity in seen:
            return
        seen.add(identity)
        merged.append(deepcopy(source))

    for source in payload_list:
        _register(source)

    def _has_used_for(sources: list[Any], target: str) -> bool:
        for source in sources:
            if not isinstance(source, dict):
                continue
            used_for = {str(item) for item in (source.get("used_for") or [])}
            if target in used_for:
                return True
        return False

    for target in ("visual_entry_optimization", "options_strategy_governance"):
        if _has_used_for(payload_list, target):
            continue
        for source in default_sources:
            used_for = {str(item) for item in (source.get("used_for") or [])}
            if target in used_for:
                _register(source)
    return merged


def normalize_trading_self_audit_payload(payload: dict[str, Any] | None) -> dict[str, Any]:
    defaults = build_trading_self_audit_note()
    if not isinstance(payload, dict):
        return defaults

    merged = deepcopy(defaults)
    for key, value in payload.items():
        if key in {
            "external_benchmark_sources",
            "visual_entry_benchmark_focus",
            "options_strategy_governance_focus",
        }:
            continue
        if value in (None, "", [], {}):
            continue
        merged[key] = deepcopy(value)

    merged["external_benchmark_sources"] = _merge_protocol_sources(
        defaults.get("external_benchmark_sources") or [],
        payload.get("external_benchmark_sources"),
    )
    merged["visual_entry_benchmark_focus"] = _merge_protocol_focus(
        defaults.get("visual_entry_benchmark_focus") or {},
        payload.get("visual_entry_benchmark_focus"),
    )
    merged["options_strategy_governance_focus"] = _merge_protocol_focus(
        defaults.get("options_strategy_governance_focus") or {},
        payload.get("options_strategy_governance_focus"),
    )
    return merged


def read_trading_self_audit_protocol(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    if not target.exists():
        return build_trading_self_audit_note()

    try:
        payload = json.loads(target.read_text(encoding="utf-8"))
    except UnicodeDecodeError:
        payload = json.loads(target.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError):
        return build_trading_self_audit_note()
    return normalize_trading_self_audit_payload(payload if isinstance(payload, dict) else {})


def persist_trading_self_audit_note(
    *,
    bridge: QuantBrainBridge | None = None,
    report_path: str | None = None,
) -> dict[str, Any]:
    note = build_trading_self_audit_note()
    if report_path:
        note["report_path"] = report_path
    memory_bridge = bridge or QuantBrainBridge()
    description = (
        "Marco logico para que ATLAS detecte, investigue, compare, corrija y siga la calidad "
        "de cada etapa del proceso de trading con el mismo metodo analitico. "
        "Foco actual: post-trade learning."
    )
    return memory_bridge.emit(
        kind="trading_self_audit_protocol",
        level="info",
        message="[QUANT][LEARNING] Protocolo logico abierto del proceso de trading registrado",
        tags=[
            "quant",
            "learning",
            "trading_process",
            "self_audit",
            "post_trade_learning",
            "pre_live",
        ],
        data=note,
        description=description,
        context=json.dumps(note, ensure_ascii=False),
        outcome="protocol_registered",
        memorize=True,
    )


def write_trading_self_audit_protocol(path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = normalize_trading_self_audit_payload(build_trading_self_audit_note())
    target.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    return target
