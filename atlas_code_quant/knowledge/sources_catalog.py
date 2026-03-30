"""Catálogo curado de referencias académicas para ATLAS-Quant.

Cada entrada sigue el esquema normalizado definido en AGENTS.md.
Campos obligatorios: id, title, authors, year, source, topic, subtopics,
timeframes, markets, method, key_findings, limitations, atlas_utility,
confidence_level, scanner_methods, tags.

Categorías:
    CAT1 — Teoría base: eficiencia, caminata aleatoria, microestructura
    CAT2 — Fenómenos de precio: momentum, reversión, autocorrelación
    CAT3 — Patrones visuales: chartismo, Elliott, H&S, tops/bottoms
    CAT4 — Modelos explicativos: conductual, risk-based, microestructura
    CAT5 — Metodología: series de tiempo, ML, validación OOS
"""
from __future__ import annotations

from typing import Any

# ── Constantes de clasificación ───────────────────────────────────────────────

TIMEFRAMES = {
    "intraday": "< 1 día",
    "short": "1–5 días",
    "swing": "5–60 días",
    "medium": "60–252 días",
    "long": "> 252 días",
    "all": "Independiente de temporalidad",
}

CONFIDENCE_LEVELS = {
    "high": "Evidencia robusta, replicada, peer-reviewed en múltiples mercados",
    "medium": "Evidencia moderada, algunos estudios de replicación, discusión en literatura",
    "low": "Evidencia preliminar, estudios únicos, o resultados mixtos",
    "contested": "Resultados contradictorios explícitos en la literatura",
}

# ── Catálogo principal ────────────────────────────────────────────────────────

SOURCES: list[dict[str, Any]] = [

    # ── CAT1: Teoría base ─────────────────────────────────────────────────────

    {
        "id": "fama_1965_rw",
        "title": "Random Walks in Stock-Market Prices",
        "authors": ["Eugene F. Fama"],
        "year": 1965,
        "source": "Financial Analysts Journal",
        "url": "https://www.jstor.org/stable/4469865",
        "category": "CAT1",
        "topic": "random_walk",
        "subtopics": ["market_efficiency", "price_prediction", "EMH"],
        "timeframes": ["all"],
        "markets": ["equity"],
        "method": "statistical_analysis",
        "key_findings": (
            "Los cambios sucesivos de precios son estadísticamente independientes bajo la hipótesis "
            "de mercado eficiente. El precio incorpora toda la información disponible de forma inmediata. "
            "La predicción sistemática de retornos futuros a partir de retornos pasados no sería posible."
        ),
        "limitations": (
            "Aplica en mercados perfectamente eficientes. En la práctica, la microestructura, los costos "
            "de transacción y los sesgos conductuales generan desviaciones explotables temporalmente."
        ),
        "atlas_utility": (
            "Base de contraste: cualquier señal del scanner debe superar el benchmark de caminata aleatoria "
            "antes de considerarse con edge real. IC del scanner mide esto directamente."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["eficiencia", "baseline", "classical", "EMH", "null_hypothesis"],
    },

    {
        "id": "ohara_1995_mmt",
        "title": "Market Microstructure Theory",
        "authors": ["Maureen O'Hara"],
        "year": 1995,
        "source": "Blackwell Publishers",
        "url": "https://www.wiley.com/en-us/Market+Microstructure+Theory-p-9780631207610",
        "category": "CAT1",
        "topic": "market_microstructure",
        "subtopics": ["order_flow", "bid_ask_spread", "price_formation", "liquidity"],
        "timeframes": ["intraday", "short"],
        "markets": ["equity", "futures"],
        "method": "theoretical_model",
        "key_findings": (
            "El proceso de negociación genera información asimétrica entre market makers y traders informados. "
            "El spread bid-ask y el impacto de precios son funciones del flujo de órdenes y la selección adversa. "
            "El precio en intraday refleja más la presión de orden que los fundamentales."
        ),
        "limitations": (
            "Aplica principalmente a horizontes intradía. En paper trading sin acceso a Level 2, "
            "los modelos de microestructura se aproximan mediante proxies (VWAP, CVD, volumen)."
        ),
        "atlas_utility": (
            "Sustenta el método order_flow_proxy del scanner. Cuando CVD diverge del precio, "
            "indica presión institucional. Relevante para gates de entrada en operaciones de corto plazo."
        ),
        "confidence_level": "high",
        "scanner_methods": ["order_flow_proxy"],
        "tags": ["microestructura", "order_flow", "intraday", "liquidity", "spread"],
    },

    {
        "id": "lo_mackinlay_1988_vr",
        "title": "Stock Market Prices Do Not Follow Random Walks",
        "authors": ["Andrew W. Lo", "A. Craig MacKinlay"],
        "year": 1988,
        "source": "Review of Financial Studies",
        "url": "https://www.jstor.org/stable/2961965",
        "category": "CAT1",
        "topic": "random_walk",
        "subtopics": ["variance_ratio", "autocorrelation", "predictability"],
        "timeframes": ["short", "swing"],
        "markets": ["equity"],
        "method": "variance_ratio_test",
        "key_findings": (
            "El test de ratio de varianza rechaza la hipótesis de caminata aleatoria para horizontes semanales. "
            "Existe autocorrelación positiva en retornos de corto plazo para acciones de pequeña capitalización. "
            "Los retornos de índices muestran mayor predictabilidad que los de acciones individuales."
        ),
        "limitations": (
            "La autocorrelación positiva puede ser artefacto de no-trading (bid-ask bounce) o "
            "de baja liquidez. No implica profit after costos de transacción."
        ),
        "atlas_utility": (
            "Valida que existen desviaciones de la caminata aleatoria explotables en horizontes cortos. "
            "Sustenta el uso de momentum de corto plazo con filtros de liquidez."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "breakout_donchian"],
        "tags": ["variance_ratio", "autocorrelation", "short_term", "predictability"],
    },

    # ── CAT2: Fenómenos de precio ─────────────────────────────────────────────

    {
        "id": "jegadeesh_titman_1993",
        "title": "Returns to Buying Winners and Selling Losers: Implications for Stock Market Efficiency",
        "authors": ["Narasimhan Jegadeesh", "Sheridan Titman"],
        "year": 1993,
        "source": "Journal of Finance",
        "url": "https://www.jstor.org/stable/2328883",
        "category": "CAT2",
        "topic": "momentum",
        "subtopics": ["cross_sectional_momentum", "winner_loser", "equity_premium"],
        "timeframes": ["swing", "medium"],
        "markets": ["equity"],
        "method": "portfolio_sort",
        "key_findings": (
            "Las acciones con mejores retornos en los últimos 3-12 meses siguen outperformando en los "
            "siguientes 3-12 meses (momentum cross-seccional). Los retornos de la estrategia no son "
            "explicables por riesgo sistemático simple (CAPM). El efecto es robusto en múltiples períodos."
        ),
        "limitations": (
            "El momentum revierte después de 12 meses (DeBondt & Thaler). Tiene crashes en periodos "
            "de recuperación post-crash. Los costos de transacción erosionan el alpha en portafolios pequeños."
        ),
        "atlas_utility": (
            "Sustenta relative_strength_overlay. Priorizar activos con mejor momentum cross-seccional "
            "en el universo scanner. Validado en períodos de 3-12 meses."
        ),
        "confidence_level": "high",
        "scanner_methods": ["relative_strength_overlay", "trend_ema_stack"],
        "tags": ["momentum", "cross_sectional", "winners", "swing", "classical"],
    },

    {
        "id": "moskowitz_ooi_pedersen_2012",
        "title": "Time Series Momentum",
        "authors": ["Tobias Moskowitz", "Yao Hua Ooi", "Lasse Heje Pedersen"],
        "year": 2012,
        "source": "Journal of Financial Economics",
        "url": "https://www.nber.org/papers/w18169.pdf",
        "category": "CAT2",
        "topic": "momentum",
        "subtopics": ["time_series_momentum", "trend_following", "multi_asset"],
        "timeframes": ["swing", "medium"],
        "markets": ["equity", "futures", "fx", "commodities"],
        "method": "portfolio_sort_time_series",
        "key_findings": (
            "Los activos con retorno positivo en los últimos 12 meses tienden a continuar en el mes siguiente "
            "(momentum temporal/absoluto). El efecto es universal en 58 instrumentos y 25 años. "
            "El retorno proviene principalmente de la tendencia del activo, no de su ranking vs. el universo."
        ),
        "limitations": (
            "Sufrió crashes significativos en 2009 y 2020. La prima de momentum puede estar ligada "
            "a riesgo de cola. Requiere diversificación para ser estable."
        ),
        "atlas_utility": (
            "Sustenta trend_ema_stack y breakout_donchian. El momentum temporal (¿el activo está subiendo?) "
            "es distinto del cross-seccional (¿está subiendo más que otros?). ATLAS usa ambos."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "breakout_donchian"],
        "tags": ["time_series_momentum", "trend", "multi_asset", "absolute_momentum"],
    },

    {
        "id": "debondt_thaler_1985",
        "title": "Does the Stock Market Overreact?",
        "authors": ["Werner F.M. De Bondt", "Richard Thaler"],
        "year": 1985,
        "source": "Journal of Finance",
        "url": "https://www.jstor.org/stable/2327804",
        "category": "CAT2",
        "topic": "mean_reversion",
        "subtopics": ["overreaction", "long_term_reversal", "behavioral"],
        "timeframes": ["long"],
        "markets": ["equity"],
        "method": "portfolio_sort",
        "key_findings": (
            "Los 'perdedores' de los últimos 3-5 años outperforman a los 'ganadores' en los 3-5 años "
            "siguientes. El mercado sobrereacciona a noticias y fundamentales. La reversión de largo plazo "
            "es opuesta al momentum de medio plazo."
        ),
        "limitations": (
            "El efecto se concentra en acciones pequeñas. Parte del retorno es atribuible a mayor riesgo. "
            "No es explotable en paper trading de corto plazo con universo US equities grande."
        ),
        "atlas_utility": (
            "Warning: el momentum de ATLAS opera en 5-60 días. La reversión de largo plazo "
            "(>1 año) no aplica directamente, pero sugiere limitar el holding en posiciones ganadoras "
            "que ya llevan más de 6-12 meses."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["mean_reversion", "long_term", "overreaction", "behavioral", "warning"],
    },

    {
        "id": "jegadeesh_1990_reversal",
        "title": "Evidence of Predictable Behavior of Security Returns",
        "authors": ["Narasimhan Jegadeesh"],
        "year": 1990,
        "source": "Journal of Finance",
        "url": "https://www.jstor.org/stable/2328880",
        "category": "CAT2",
        "topic": "short_term_reversal",
        "subtopics": ["weekly_reversal", "bid_ask_bounce", "microstructure"],
        "timeframes": ["intraday", "short"],
        "markets": ["equity"],
        "method": "portfolio_sort",
        "key_findings": (
            "Los retornos a 1 semana muestran autocorrelación negativa (reversión). "
            "Los activos que más subieron esta semana tienden a bajar la siguiente. "
            "El efecto es parcialmente explicado por bid-ask bounce y microestructura."
        ),
        "limitations": (
            "El efecto de reversión de 1 semana es frágil fuera de sample. Muy sensible a "
            "costos de transacción. No recomendado para estrategias con holding > 2 días."
        ),
        "atlas_utility": (
            "Sustenta rsi_pullback_trend: el pullback dentro de tendencia explota la reversión "
            "de corto plazo pero solo cuando la tendencia dominante sigue activa. Evitar reversión pura."
        ),
        "confidence_level": "medium",
        "scanner_methods": ["rsi_pullback_trend"],
        "tags": ["reversal", "short_term", "weekly", "bid_ask", "fragile"],
    },

    {
        "id": "asness_moskowitz_pedersen_2013",
        "title": "Value and Momentum Everywhere",
        "authors": ["Clifford Asness", "Tobias Moskowitz", "Lasse Pedersen"],
        "year": 2013,
        "source": "Journal of Finance",
        "url": "https://www.jstor.org/stable/43612950",
        "category": "CAT2",
        "topic": "momentum",
        "subtopics": ["value_momentum", "multi_asset", "IC_methodology"],
        "timeframes": ["swing", "medium"],
        "markets": ["equity", "fx", "futures", "bonds"],
        "method": "IC_analysis_portfolio_sort",
        "key_findings": (
            "Momentum y value funcionan universalmente en 8 clases de activos y 4 países. "
            "Están negativamente correlacionados entre sí, ofreciendo diversificación real. "
            "El IC del momentum es robusto y replicable out-of-sample. "
            "Valida la metodología IC de Grinold-Kahn en contextos multi-activo."
        ),
        "limitations": (
            "Los retornos combinados value+momentum son menores en períodos recientes post-QE. "
            "El value solo ha underperformado significativamente en la última década."
        ),
        "atlas_utility": (
            "Validación directa de la metodología IC del ICSignalTracker. La robustez del IC "
            "de momentum en múltiples activos respalda el uso de predicted_move_pct como señal. "
            "Referencia principal para interpretar IC > 0.05 como meaningful."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "relative_strength_overlay"],
        "tags": ["IC_validation", "momentum", "value", "multi_asset", "grinold_kahn"],
    },

    # ── CAT3: Patrones visuales ───────────────────────────────────────────────

    {
        "id": "lo_mamaysky_wang_2000",
        "title": "Foundations of Technical Analysis: Computational Algorithms, Statistical Inference, and Empirical Implementation",
        "authors": ["Andrew W. Lo", "Harry Mamaysky", "Jiang Wang"],
        "year": 2000,
        "source": "Journal of Finance",
        "url": "https://www.jstor.org/stable/222453",
        "category": "CAT3",
        "topic": "technical_patterns",
        "subtopics": ["head_and_shoulders", "double_top_bottom", "pattern_recognition"],
        "timeframes": ["short", "swing"],
        "markets": ["equity"],
        "method": "nonparametric_kernel_regression",
        "key_findings": (
            "Algunos patrones chartistas clásicos (head-and-shoulders, double top/bottom, triangles) "
            "contienen información estadísticamente significativa sobre retornos futuros. "
            "Los patrones no pueden explicarse completamente por la hipótesis de caminata aleatoria. "
            "El head-and-shoulders tiene evidencia más robusta que otros patrones."
        ),
        "limitations": (
            "La identificación objetiva de patrones es difícil. Los retornos son pequeños y sensibles "
            "a costos de transacción. Los patrones no son confiables de forma aislada."
        ),
        "atlas_utility": (
            "Sustenta el uso de patrones visuales en sensor_vision y chart_ocr. "
            "La validación empírica de H&S refuerza el uso de exit governance cuando se detecta "
            "un patrón de distribución en una posición abierta."
        ),
        "confidence_level": "medium",
        "scanner_methods": [],
        "tags": ["chart_patterns", "head_and_shoulders", "technical_analysis", "visual"],
    },

    {
        "id": "prechter_frost_1978_elliott",
        "title": "Elliott Wave Principle — Key to Market Behavior",
        "authors": ["Robert Prechter", "A.J. Frost"],
        "year": 1978,
        "source": "New Classics Library",
        "url": "https://elliottwave.com/free-updates/educational/elliott-wave-principle-key-to-market-behavior/",
        "category": "CAT3",
        "topic": "elliott_waves",
        "subtopics": ["wave_count", "fractal_structure", "market_cycles"],
        "timeframes": ["all"],
        "markets": ["equity", "fx", "commodities"],
        "method": "pattern_classification",
        "key_findings": (
            "Los mercados siguen una estructura fractal de 5 ondas impulsivas y 3 correctivas. "
            "La cuenta de onda permite identificar la posición en el ciclo y proyectar objetivos. "
            "La estructura se replica en múltiples temporalidades simultáneamente."
        ),
        "limitations": (
            "El conteo de ondas es altamente subjetivo — distintos analistas llegan a conclusiones "
            "opuestas con el mismo gráfico. La evidencia empírica de profitabilidad es débil. "
            "Ver: Poser (2003) que revisa la efectividad con evidencia mixta."
        ),
        "atlas_utility": (
            "WARNING: baja confianza empírica para trading algorítmico. Útil como marco conceptual "
            "para identificar estructuras de mercado, pero NO como señal de entrada autónoma. "
            "Usar solo como contexto de chart_plan visual, no como trigger."
        ),
        "confidence_level": "low",
        "scanner_methods": [],
        "tags": ["elliott", "waves", "fractal", "subjective", "warning", "low_confidence"],
    },

    {
        "id": "poser_2003_elliott_effectiveness",
        "title": "The Effectiveness of the Elliott Waves Theory to Forecast Financial Markets",
        "authors": ["Stanley Poser"],
        "year": 2003,
        "source": "ABI/Inform",
        "url": "",
        "category": "CAT3",
        "topic": "elliott_waves",
        "subtopics": ["empirical_validation", "forecasting", "elliott_critique"],
        "timeframes": ["all"],
        "markets": ["equity"],
        "method": "empirical_review",
        "key_findings": (
            "La revisión empírica de la teoría de Elliott muestra resultados mixtos. "
            "El éxito reportado es en gran parte anecdótico. La precisión de las predicciones "
            "basadas en ondas no supera significativamente al azar en pruebas objetivas."
        ),
        "limitations": (
            "El estudio mismo reconoce dificultad en formalizar el conteo de ondas para "
            "backtesting riguroso, lo que limita las conclusiones negativas también."
        ),
        "atlas_utility": (
            "Referencia crítica que modera el uso de Elliott en ATLAS. Confirma que el conteo "
            "de ondas no debe ser señal de entrada primaria. Solo como contexto de estructura."
        ),
        "confidence_level": "medium",
        "scanner_methods": [],
        "tags": ["elliott", "empirical", "critique", "contested", "forecasting"],
    },

    # ── CAT4: Modelos explicativos ────────────────────────────────────────────

    {
        "id": "daniel_hirshleifer_subrahmanyam_1998",
        "title": "Investor Psychology and Security Market Under- and Overreactions",
        "authors": ["Kent Daniel", "David Hirshleifer", "Avanidhar Subrahmanyam"],
        "year": 1998,
        "source": "Journal of Finance",
        "url": "https://www.jstor.org/stable/117429",
        "category": "CAT4",
        "topic": "behavioral_finance",
        "subtopics": ["overconfidence", "self_attribution", "momentum_explanation"],
        "timeframes": ["swing", "medium", "long"],
        "markets": ["equity"],
        "method": "theoretical_model",
        "key_findings": (
            "El momentum de medio plazo y la reversión de largo plazo son explicables por sobreconfianza "
            "y sesgo de autoatribución. Los inversores sobrereaccionan a señales privadas y subreaccionan "
            "a señales públicas. El modelo predice momentum seguido de reversión."
        ),
        "limitations": (
            "Es un modelo explicativo, no predictivo. La calibración del horizonte temporal "
            "varía entre mercados. No genera señales operativas directas."
        ),
        "atlas_utility": (
            "Explica por qué la cadena trend_ema_stack → relative_strength puede tener edge: "
            "los inversores sobrereaccionan a señales privadas (generando momentum) y luego corrigen. "
            "Sugiere tomar profits antes de la reversión (exit governance activo)."
        ),
        "confidence_level": "medium",
        "scanner_methods": ["trend_ema_stack", "rsi_pullback_trend"],
        "tags": ["behavioral", "overconfidence", "momentum_explanation", "reversal_warning"],
    },

    {
        "id": "barberis_thaler_2003",
        "title": "A Survey of Behavioral Finance",
        "authors": ["Nicholas Barberis", "Richard Thaler"],
        "year": 2003,
        "source": "Handbook of the Economics of Finance",
        "url": "https://www.nber.org/papers/w9222",
        "category": "CAT4",
        "topic": "behavioral_finance",
        "subtopics": ["limits_to_arbitrage", "cognitive_biases", "asset_pricing_anomalies"],
        "timeframes": ["all"],
        "markets": ["equity"],
        "method": "literature_survey",
        "key_findings": (
            "Dos pilares: (1) límites al arbitraje — los precios pueden desviarse sin corrección inmediata; "
            "(2) sesgos cognitivos — sobreconfianza, representatividad, anclaje y efecto de disposición "
            "generan patrones predecibles. El efecto de disposición (vender ganadores demasiado pronto, "
            "mantener perdedores) es una de las anomalías más documentadas."
        ),
        "limitations": (
            "Los límites al arbitraje son menores en activos líquidos de gran capitalización. "
            "Los sesgos son más fuertes en retail que en institucional."
        ),
        "atlas_utility": (
            "Justifica el exit_governance_snapshot: los inversores retail tienden a vender ganadores "
            "rápido y mantener perdedores. ATLAS debe hacer lo opuesto: cortar pérdidas rápido "
            "y dejar correr ganadores mientras la tendencia siga activa."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["behavioral", "survey", "limits_arbitrage", "disposition_effect", "exit_governance"],
    },

    {
        "id": "grinold_kahn_2000_apm",
        "title": "Active Portfolio Management: A Quantitative Approach for Producing Superior Returns",
        "authors": ["Richard Grinold", "Ronald Kahn"],
        "year": 2000,
        "source": "McGraw-Hill",
        "url": "https://www.mhprofessional.com/active-portfolio-management-9780070248823-usa",
        "category": "CAT4",
        "topic": "signal_quality",
        "subtopics": ["information_coefficient", "information_ratio", "breadth", "fundamental_law"],
        "timeframes": ["all"],
        "markets": ["equity"],
        "method": "quantitative_framework",
        "key_findings": (
            "IR ≈ IC × √Breadth (Ley Fundamental de la Gestión Activa). "
            "IC > 0.05 es meaningful; IC > 0.10 es fuerte. t-stat(IC) ≥ 2.0 requerido para producción. "
            "Más señales independientes (breadth) con IC modesto pueden superar a una señal con IC alto."
        ),
        "limitations": (
            "Asume IC estable en el tiempo — en la práctica el IC varía por régimen. "
            "La estimación del IC requiere muestra suficiente (n ≥ 30)."
        ),
        "atlas_utility": (
            "Marco central del ICSignalTracker. Define los umbrales operativos de IC para ATLAS. "
            "El ICSignalTracker implementa exactamente la metodología IC de Grinold-Kahn."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "breakout_donchian", "rsi_pullback_trend", "ml_directional", "relative_strength_overlay"],
        "tags": ["IC", "information_ratio", "signal_quality", "fundamental_law", "production_criteria"],
    },

    {
        "id": "perold_1988_is",
        "title": "The Implementation Shortfall: Paper versus Reality",
        "authors": ["Andre Perold"],
        "year": 1988,
        "source": "Journal of Portfolio Management",
        "url": "https://www.pm-research.com/content/iijpormgmt/14/3/4",
        "category": "CAT4",
        "topic": "execution_quality",
        "subtopics": ["implementation_shortfall", "transaction_costs", "slippage", "market_impact"],
        "timeframes": ["intraday", "short"],
        "markets": ["equity"],
        "method": "decomposition_framework",
        "key_findings": (
            "El costo real de trading (Implementation Shortfall) = retorno en papel – retorno real. "
            "Se descompone en: delay cost, market impact, spread cost, opportunity cost. "
            "Los costos de ejecución pueden eliminar completamente el alpha de señales débiles."
        ),
        "limitations": (
            "En paper trading los costos son cero, pero en live trading pueden ser 0.1-0.5% por operación. "
            "Señales con IC débil (< 0.05) son especialmente vulnerables."
        ),
        "atlas_utility": (
            "Define el estándar de ejecución para el loop autónomo. Justifica medir el tiempo "
            "entre señal y orden (decision lag). Señales con IC < 0.05 NO justifican costos de ejecución real."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["execution", "implementation_shortfall", "transaction_costs", "live_criteria"],
    },

    # ── CAT5: Metodología ─────────────────────────────────────────────────────

    {
        "id": "tsay_2010_fts",
        "title": "Analysis of Financial Time Series",
        "authors": ["Ruey S. Tsay"],
        "year": 2010,
        "source": "Wiley",
        "url": "https://www.wiley.com/en-us/Analysis+of+Financial+Time+Series%2C+3rd+Edition-p-9780470414354",
        "category": "CAT5",
        "topic": "time_series_analysis",
        "subtopics": ["ARIMA", "GARCH", "volatility_modeling", "autocorrelation", "heteroskedasticity"],
        "timeframes": ["short", "swing", "medium"],
        "markets": ["equity", "fx"],
        "method": "econometric_modeling",
        "key_findings": (
            "Los retornos financieros muestran heterocedasticidad condicional (ARCH/GARCH). "
            "La volatilidad es cluster: períodos de alta volatilidad siguen a períodos de alta volatilidad. "
            "Los modelos GARCH mejoran la estimación de riesgo y son útiles para sizing dinámico."
        ),
        "limitations": (
            "Los modelos ARIMA/GARCH capturan dependencias lineales. Patrones no-lineales "
            "requieren modelos más complejos. En alta frecuencia, la microestructura contamina los residuos."
        ),
        "atlas_utility": (
            "Base para el KellyRiskEngine: la volatilidad condicional (GARCH-like) mejora el sizing. "
            "Justifica ATR como proxy de volatilidad en horizon corto. "
            "El clustering de volatilidad sugiere reducir tamaño en períodos de VIX elevado."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "breakout_donchian"],
        "tags": ["GARCH", "ARIMA", "volatility", "time_series", "sizing"],
    },

    {
        "id": "lo_mackinlay_1990_overreactions",
        "title": "When Are Contrarian Profits Due to Stock Market Overreaction?",
        "authors": ["Andrew W. Lo", "A. Craig MacKinlay"],
        "year": 1990,
        "source": "Review of Financial Studies",
        "url": "https://www.jstor.org/stable/2962020",
        "category": "CAT5",
        "topic": "contrarian_momentum",
        "subtopics": ["cross_autocorrelation", "lead_lag", "portfolio_attribution"],
        "timeframes": ["short", "swing"],
        "markets": ["equity"],
        "method": "portfolio_decomposition",
        "key_findings": (
            "Los retornos de estrategias contrarias no son completamente explicados por sobrereacción. "
            "La autocorrelación cruzada entre acciones (lead-lag de grandes a pequeñas) contribuye "
            "significativamente a los retornos contrarios. El momentum puede ser explota este lead-lag."
        ),
        "limitations": (
            "El lead-lag es menos prominente en mercados modernos con algoritmos que integran "
            "información más rápido. Puede ser artefacto de sincronismo de precios."
        ),
        "atlas_utility": (
            "Sustenta el uso de índices y ETFs como leading indicators del universo de acciones. "
            "Cuando SPY/QQQ muestran momentum, las acciones del universo lo siguen con lag."
        ),
        "confidence_level": "medium",
        "scanner_methods": ["relative_strength_overlay", "trend_ema_stack"],
        "tags": ["lead_lag", "cross_autocorrelation", "contrarian", "index_leading"],
    },

    {
        "id": "lopez_de_prado_2018_afml",
        "title": "Advances in Financial Machine Learning",
        "authors": ["Marcos Lopez de Prado"],
        "year": 2018,
        "source": "Wiley",
        "url": "https://www.wiley.com/en-us/Advances+in+Financial+Machine+Learning-p-9781119482086",
        "category": "CAT5",
        "topic": "ml_finance",
        "subtopics": ["feature_engineering", "backtesting_pitfalls", "deflated_sharpe", "MTRL", "bars_alternatives"],
        "timeframes": ["all"],
        "markets": ["equity", "futures", "fx"],
        "method": "ml_framework",
        "key_findings": (
            "La mayoría de backtests en finanzas sufren de múltiples testing, selección de muestra "
            "y look-ahead bias. El Deflated Sharpe Ratio ajusta por overfitting y número de pruebas. "
            "Bars de tick/volume/dollar son superiores a bars de tiempo para ML en finanzas. "
            "MTRL: mínimo 12 meses OOS para evaluar un sistema antes de ir a live."
        ),
        "limitations": (
            "La implementación práctica requiere infraestructura de datos robusta. "
            "Las barras alternativas no siempre están disponibles en APIs retail."
        ),
        "atlas_utility": (
            "Marco para la decisión live/no-live. Define los criterios del TSAP: "
            "Deflated Sharpe > 0, MTRL >= 3 meses OOS, n_trades suficiente. "
            "Justifica no ir a live antes de 6 meses de paper con evidencia real."
        ),
        "confidence_level": "high",
        "scanner_methods": ["ml_directional"],
        "tags": ["ML", "backtesting", "deflated_sharpe", "MTRL", "live_criteria", "overfitting"],
    },

    {
        "id": "van_tharp_1999_riskmanagement",
        "title": "Trade Your Way to Financial Freedom",
        "authors": ["Van K. Tharp"],
        "year": 1999,
        "source": "McGraw-Hill",
        "url": "https://www.mhprofessional.com/trade-your-way-to-financial-freedom-9780071493208-usa",
        "category": "CAT5",
        "topic": "position_sizing",
        "subtopics": ["R_multiples", "expectancy", "risk_per_trade", "position_sizing"],
        "timeframes": ["all"],
        "markets": ["equity", "futures", "fx"],
        "method": "risk_framework",
        "key_findings": (
            "El tamaño de posición (position sizing) es más determinante que el sistema de señales "
            "para el resultado final. R-múltiple = retorno / riesgo inicial definido por stop. "
            "Expectancy = E[R] debe ser > 0 para que el sistema sea ganador. "
            "Sistemas con Expectancy = 0.5R generan compounding robusto."
        ),
        "limitations": (
            "Requiere definición objetiva del stop antes de entrar. Sin stop definido, "
            "el R-múltiple no puede calcularse y el framework pierde sentido."
        ),
        "atlas_utility": (
            "Marco central de position_management_snapshot y exit_governance. "
            "Justifica la integración de StrategySelector con stop_pct en el auto-cycle. "
            "Sin stop en el journal, los R-múltiples son indefinidos — gap crítico identificado."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "breakout_donchian", "rsi_pullback_trend"],
        "tags": ["R_multiples", "position_sizing", "expectancy", "stop_loss", "risk_management"],
    },

    {
        "id": "kaminski_lo_2014_stoploss",
        "title": "When Do Stop-Loss Rules Stop Losses?",
        "authors": ["Kathryn Kaminski", "Andrew Lo"],
        "year": 2014,
        "source": "Journal of Financial Markets",
        "url": "https://www.sciencedirect.com/science/article/pii/S1386418113000645",
        "category": "CAT5",
        "topic": "stop_loss",
        "subtopics": ["stop_loss_effectiveness", "drawdown_control", "regime_dependent"],
        "timeframes": ["short", "swing"],
        "markets": ["equity", "futures"],
        "method": "empirical_simulation",
        "key_findings": (
            "Los stops basados en reglas sistemáticas (volatilidad-adaptive) reducen drawdown sin "
            "sacrificar retorno esperado en mercados con momentum. Los stops fijos degradan el "
            "rendimiento en mercados de alta volatilidad. La efectividad depende del régimen: "
            "stops son más útiles en mercados trending que en ranging."
        ),
        "limitations": (
            "En mercados con gap overnight, los stops no protegen completamente. "
            "En opciones, el stop del subyacente no equivale al stop de la posición."
        ),
        "atlas_utility": (
            "Valida el uso de ATR como base para stops dinámicos en el StrategySelector. "
            "Sugiere adaptar el stop según el régimen (RegimeClassifier output). "
            "En régimen trending: stop más amplio. En ranging: stop más estrecho o no operar."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack", "breakout_donchian"],
        "tags": ["stop_loss", "ATR", "regime_adaptive", "drawdown", "exit_governance"],
    },

    {
        "id": "almgren_chriss_2001_execution",
        "title": "Optimal Execution of Portfolio Transactions",
        "authors": ["Robert Almgren", "Neil Chriss"],
        "year": 2001,
        "source": "Journal of Risk",
        "url": "https://www.risk.net/journal-of-risk/1509161/optimal-execution-of-portfolio-transactions",
        "category": "CAT5",
        "topic": "execution_quality",
        "subtopics": ["optimal_execution", "market_impact", "VWAP", "risk_tradeoff"],
        "timeframes": ["intraday"],
        "markets": ["equity", "futures"],
        "method": "optimization_model",
        "key_findings": (
            "La ejecución óptima minimiza el trade-off entre impacto de mercado y riesgo de timing. "
            "Para ordenes pequeñas (< 0.1% del volumen diario), el VWAP es una estrategia casi óptima. "
            "El impacto de mercado es lineal en el tamaño de la orden para operaciones pequeñas."
        ),
        "limitations": (
            "El modelo aplica a ejecución institucional. En paper trading con size=1 acción, "
            "el impacto es cero. Relevante cuando ATLAS escale a tamaños mayores."
        ),
        "atlas_utility": (
            "Para el estado actual de ATLAS (size=1 acción), el impacto es negligible. "
            "Referencia para cuando se escale el sistema: size no debe superar 0.1% del ADV. "
            "Justifica usar VWAP como precio de referencia en el implementation shortfall."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["execution", "VWAP", "market_impact", "optimal_execution", "scaling"],
    },

    {
        "id": "brinson_hood_beebower_1986",
        "title": "Determinants of Portfolio Performance",
        "authors": ["Gary Brinson", "L. Randolph Hood", "Gilbert Beebower"],
        "year": 1986,
        "source": "Financial Analysts Journal",
        "url": "https://www.jstor.org/stable/4478947",
        "category": "CAT5",
        "topic": "portfolio_attribution",
        "subtopics": ["BHB_attribution", "asset_allocation", "security_selection", "timing"],
        "timeframes": ["medium", "long"],
        "markets": ["equity", "bonds"],
        "method": "attribution_decomposition",
        "key_findings": (
            "El 93.6% de la variación de retornos entre carteras se explica por la asignación "
            "de activos (asset allocation). La selección de valores y el timing contribuyen marginalmente. "
            "La decisión de en qué clase de activo estar es más importante que qué activo elegir."
        ),
        "limitations": (
            "El estudio aplica a carteras diversificadas institucionales. En un sistema de short-term "
            "trading como ATLAS, la selección de valor y el timing son más relevantes que la asignación."
        ),
        "atlas_utility": (
            "Marco de post_trade_learning_snapshot. La atribución BHB identifica si el alpha "
            "viene de la clase de activo (equity vs options vs ETF) o del símbolo específico. "
            "Requiere strategy_type correctamente atribuido para cada trade cerrado."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["attribution", "BHB", "post_trade", "asset_allocation", "learning"],
    },

    {
        "id": "faber_2007_tactical",
        "title": "A Quantitative Approach to Tactical Asset Allocation",
        "authors": ["Mebane Faber"],
        "year": 2007,
        "source": "SSRN",
        "url": "https://mebfaber.com/wp-content/uploads/2010/10/SSRN-id962461.pdf",
        "category": "CAT2",
        "topic": "trend_following",
        "subtopics": ["moving_average", "tactical_allocation", "market_timing"],
        "timeframes": ["medium"],
        "markets": ["equity", "bonds", "commodities", "fx"],
        "method": "backtesting_rule_based",
        "key_findings": (
            "Una regla simple de media móvil de 10 meses (SMA 200 días) mejora el Sharpe y reduce "
            "drawdown en múltiples clases de activos. El filtro de tendencia no requiere predicción: "
            "solo determina si el precio está sobre o bajo la media. Out-of-sample: Sharpe ~0.7 vs ~0.4 buy&hold."
        ),
        "limitations": (
            "Basado en datos mensuales — aplica a horizontes de medio/largo plazo. "
            "El whipsaw en mercados laterales erosiona el retorno. Requiere patience."
        ),
        "atlas_utility": (
            "Sustenta la lógica de trend_ema_stack como filtro de régimen: solo operar en dirección "
            "de la tendencia estructural (precio > EMA 200). El filtro no predice, selecciona lado."
        ),
        "confidence_level": "high",
        "scanner_methods": ["trend_ema_stack"],
        "tags": ["trend_following", "moving_average", "market_timing", "drawdown_reduction"],
    },

    {
        "id": "black_scholes_1973",
        "title": "The Pricing of Options and Corporate Liabilities",
        "authors": ["Fischer Black", "Myron Scholes"],
        "year": 1973,
        "source": "Journal of Political Economy",
        "url": "https://www.jstor.org/stable/1831029",
        "category": "CAT4",
        "topic": "options_pricing",
        "subtopics": ["Black_Scholes", "implied_volatility", "delta", "options_valuation"],
        "timeframes": ["all"],
        "markets": ["equity", "options"],
        "method": "analytical_model",
        "key_findings": (
            "El precio de una opción es función de: precio del subyacente, strike, tiempo al vencimiento, "
            "tasa libre de riesgo y volatilidad implícita (IV). La IV es la única variable no observable "
            "directamente — el mercado la cotiza en el precio de la opción. "
            "IV alta = opciones caras = vendedores tienen edge estadístico en el tiempo."
        ),
        "limitations": (
            "Asume volatilidad constante (skew = 0) y distribución log-normal. "
            "En la práctica existe volatility smile/skew que el modelo no captura."
        ),
        "atlas_utility": (
            "Base del OptionSelector y StrategySelector. IV rank (IV actual vs histórica) determina "
            "si conviene comprar o vender opciones. El sistema OptionStrat implementa este principio."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["options", "IV", "Black_Scholes", "options_pricing", "options_strategy"],
    },

    {
        "id": "natenberg_1994_options_volatility",
        "title": "Option Volatility and Pricing",
        "authors": ["Sheldon Natenberg"],
        "year": 1994,
        "source": "McGraw-Hill",
        "url": "https://www.mhprofessional.com/option-volatility-and-pricing-9780071818773-usa",
        "category": "CAT4",
        "topic": "options_trading",
        "subtopics": ["volatility_trading", "IV_rank", "delta_hedging", "options_strategies"],
        "timeframes": ["short", "swing"],
        "markets": ["options"],
        "method": "practitioner_framework",
        "key_findings": (
            "La volatilidad implícita revierte a la media — las opciones caras (IV alta) tienden "
            "a bajar en volatilidad y vice versa. Vender premium cuando IV rank > 50% tiene edge "
            "estadístico documentado en múltiples estudios empíricos."
        ),
        "limitations": (
            "La reversión a la media de IV puede fallar en eventos de tail risk. "
            "La gestión de riesgo (delta-hedging, max loss) es crítica."
        ),
        "atlas_utility": (
            "Marco operativo del OptionSelector: IV rank > 50% → considerar venta de premium. "
            "IV rank < 30% → considerar compra de opciones / spreads debit. "
            "DTE 21-45 días: zona óptima para theta decay según el framework."
        ),
        "confidence_level": "high",
        "scanner_methods": [],
        "tags": ["options", "IV_rank", "premium_selling", "theta", "options_strategy"],
    },
]

# ── Índice por categorías ─────────────────────────────────────────────────────

CATEGORY_LABELS = {
    "CAT1": "Teoría base — eficiencia, caminata aleatoria, microestructura",
    "CAT2": "Fenómenos de precio — momentum, reversión, tendencia",
    "CAT3": "Patrones visuales — chartismo, Elliott, H&S",
    "CAT4": "Modelos explicativos — conductual, risk-based, opciones",
    "CAT5": "Metodología — series de tiempo, ML, validación, ejecución",
}

# ── Mapa método scanner → fuentes relevantes ─────────────────────────────────

SCANNER_METHOD_TO_SOURCES: dict[str, list[str]] = {}
for _src in SOURCES:
    for _method in (_src.get("scanner_methods") or []):
        SCANNER_METHOD_TO_SOURCES.setdefault(_method, []).append(_src["id"])

# ── Mapa temporalidad → fuentes ───────────────────────────────────────────────

TIMEFRAME_TO_SOURCES: dict[str, list[str]] = {}
for _src in SOURCES:
    for _tf in (_src.get("timeframes") or []):
        TIMEFRAME_TO_SOURCES.setdefault(_tf, []).append(_src["id"])
