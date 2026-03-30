"""Motor de consulta de la biblioteca académica de ATLAS-Quant.

La KnowledgeBase es consultable por:
  - método del scanner (cuál literatura respalda trend_ema_stack, etc.)
  - temporalidad (intraday, swing, medium, long)
  - tema (momentum, reversal, microstructure, etc.)
  - nivel de confianza
  - búsqueda libre (search)

El método principal es advisory_context(), que devuelve un dict con:
  - fuentes relevantes con hallazgos clave
  - warnings si la señal tiene evidencia débil o contestada
  - recomendaciones operativas derivadas de la literatura
  - nivel de soporte académico global (0-100)
  - ic_live_data: IC real del ICSignalTracker para el método consultado

El advisory_context es NO BLOQUEANTE: se adjunta como contexto informativo
al resultado del OperationCenter, no como gate de bloqueo.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from atlas_code_quant.knowledge.sources_catalog import (
    SOURCES,
    SCANNER_METHOD_TO_SOURCES,
    TIMEFRAME_TO_SOURCES,
    CATEGORY_LABELS,
)

# Mapa id → fuente completa para lookup O(1)
_SOURCE_BY_ID: dict[str, dict[str, Any]] = {s["id"]: s for s in SOURCES}

# Mapa normalización de temporalidades del scanner → catálogo
_TIMEFRAME_ALIASES: dict[str, str] = {
    "1m": "intraday",
    "5m": "intraday",
    "15m": "intraday",
    "30m": "intraday",
    "1h": "short",
    "4h": "short",
    "1d": "swing",
    "1w": "medium",
    "1mo": "long",
}

# Mapa tema → palabras clave en subtopics/tags
_TOPIC_KEYWORDS: dict[str, list[str]] = {
    "momentum": ["momentum", "trend", "winners", "time_series_momentum"],
    "mean_reversion": ["mean_reversion", "reversal", "overreaction", "contrarian"],
    "microstructure": ["microestructura", "order_flow", "bid_ask", "microstructure", "spread"],
    "behavioral": ["behavioral", "overconfidence", "disposition_effect", "cognitive_biases"],
    "options": ["options", "IV", "Black_Scholes", "IV_rank", "premium_selling"],
    "ml": ["ML", "machine_learning", "ml_finance", "feature_engineering"],
    "execution": ["execution", "implementation_shortfall", "VWAP", "market_impact"],
    "risk": ["R_multiples", "position_sizing", "stop_loss", "drawdown", "risk_management"],
    "attribution": ["attribution", "BHB", "post_trade", "asset_allocation"],
    "signal_quality": ["IC", "information_ratio", "signal_quality", "IC_validation"],
}


class KnowledgeBase:
    """Biblioteca académica consultable para toma de decisiones de trading.

    No persiste estado — es stateless y puede instanciarse sin I/O.
    El catálogo vive en sources_catalog.py y se carga en memoria.
    """

    def __init__(self) -> None:
        self._sources = SOURCES
        self._by_id = _SOURCE_BY_ID

    # ── Consulta por método scanner ───────────────────────────────────────────

    def sources_for_method(self, method: str) -> list[dict[str, Any]]:
        """Retorna fuentes relevantes para un método del scanner."""
        ids = SCANNER_METHOD_TO_SOURCES.get(str(method), [])
        return [self._by_id[sid] for sid in ids if sid in self._by_id]

    # ── Consulta por temporalidad ─────────────────────────────────────────────

    def sources_for_timeframe(self, timeframe: str) -> list[dict[str, Any]]:
        """Retorna fuentes relevantes para una temporalidad (1d, swing, intraday, etc.)."""
        tf_normalized = _TIMEFRAME_ALIASES.get(str(timeframe).lower(), str(timeframe).lower())
        ids = (
            TIMEFRAME_TO_SOURCES.get(tf_normalized, [])
            + TIMEFRAME_TO_SOURCES.get("all", [])
        )
        seen: set[str] = set()
        result = []
        for sid in ids:
            if sid not in seen and sid in self._by_id:
                seen.add(sid)
                result.append(self._by_id[sid])
        return result

    # ── Consulta por tema ─────────────────────────────────────────────────────

    def sources_for_topic(self, topic: str) -> list[dict[str, Any]]:
        """Retorna fuentes relevantes para un tema (momentum, reversal, options, etc.)."""
        keywords = _TOPIC_KEYWORDS.get(str(topic).lower(), [str(topic).lower()])
        result = []
        for src in self._sources:
            tags = [str(t).lower() for t in (src.get("tags") or [])]
            subtopics = [str(s).lower() for s in (src.get("subtopics") or [])]
            combined = tags + subtopics + [str(src.get("topic", "")).lower()]
            if any(kw.lower() in " ".join(combined) for kw in keywords):
                result.append(src)
        return result

    # ── Consulta por nivel de confianza ──────────────────────────────────────

    def sources_by_confidence(self, min_level: str = "medium") -> list[dict[str, Any]]:
        """Filtra fuentes por nivel mínimo de confianza."""
        order = {"high": 3, "medium": 2, "low": 1, "contested": 0}
        min_score = order.get(min_level, 0)
        return [
            s for s in self._sources
            if order.get(str(s.get("confidence_level", "low")), 0) >= min_score
        ]

    # ── Contexto advisory principal ───────────────────────────────────────────

    def advisory_context(
        self,
        *,
        method: str | None = None,
        timeframe: str | None = None,
        topic: str | None = None,
        max_sources: int = 5,
    ) -> dict[str, Any]:
        """Genera contexto advisory para una decisión de trading.

        Combina fuentes relevantes por método + temporalidad y computa:
        - academic_support_score (0–100): soporte académico agregado
        - warnings: cuando la señal tiene evidencia débil o contestada
        - key_insights: hallazgos clave de las fuentes más relevantes
        - recommended_considerations: acciones operativas derivadas

        Args:
            method: Método del scanner (e.g., "trend_ema_stack").
            timeframe: Temporalidad (e.g., "1d", "swing", "intraday").
            topic: Tema adicional para enriquecer el contexto.
            max_sources: Máximo de fuentes a incluir en el contexto.

        Returns:
            Dict con advisory context listo para adjuntar al resultado del OperationCenter.
        """
        # Recolectar fuentes relevantes
        found: dict[str, dict[str, Any]] = {}

        if method:
            for src in self.sources_for_method(method):
                found[src["id"]] = src

        if timeframe:
            for src in self.sources_for_timeframe(timeframe):
                # Priorizar fuentes que también apliquen al método
                if src["id"] not in found:
                    found[src["id"]] = src

        if topic:
            for src in self.sources_for_topic(topic):
                if src["id"] not in found:
                    found[src["id"]] = src

        # Ordenar por: (1) método-match primero, (2) confianza, (3) año desc
        confidence_order = {"high": 3, "medium": 2, "low": 1, "contested": 0}
        method_ids = set(SCANNER_METHOD_TO_SOURCES.get(str(method or ""), []))

        ranked = sorted(
            found.values(),
            key=lambda s: (
                1 if s["id"] in method_ids else 0,
                confidence_order.get(str(s.get("confidence_level", "low")), 0),
                int(s.get("year", 0)),
            ),
            reverse=True,
        )[:max_sources]

        # Computar academic_support_score
        if not ranked:
            support_score = 0.0
        else:
            score_map = {"high": 1.0, "medium": 0.65, "low": 0.3, "contested": 0.1}
            raw_scores = [score_map.get(str(s.get("confidence_level", "low")), 0.3) for s in ranked]
            support_score = round(sum(raw_scores) / len(raw_scores) * 100.0, 1)

        # Detectar warnings
        warnings: list[str] = []
        for src in ranked:
            conf = str(src.get("confidence_level", ""))
            if conf == "low":
                warnings.append(
                    f"[{src['id']}] Evidencia baja: '{src['title']}' — "
                    f"usar solo como contexto, no como señal primaria."
                )
            if conf == "contested":
                warnings.append(
                    f"[{src['id']}] Evidencia contestada: '{src['title']}' — "
                    f"resultados mixtos en la literatura."
                )
            if "warning" in (src.get("tags") or []):
                warnings.append(
                    f"[{src['id']}] WARNING en atlas_utility: {src.get('atlas_utility', '')[:150]}"
                )

        # Extraer insights clave
        key_insights = [
            {
                "id": src["id"],
                "title": src["title"],
                "authors": src.get("authors", []),
                "year": src.get("year"),
                "confidence": src.get("confidence_level"),
                "finding": src.get("key_findings", "")[:300],
                "atlas_utility": src.get("atlas_utility", "")[:300],
                "limitations": src.get("limitations", "")[:200],
            }
            for src in ranked
        ]

        # Recomendaciones operativas por método
        recommendations = _build_recommendations(method, timeframe, ranked)

        # IC live data del ICSignalTracker
        ic_live = _read_ic_live_data(method)
        if ic_live and ic_live.get("n_observations", 0) == 0 and method:
            warnings.append(
                f"[ic_tracker] Sin observaciones OOS para '{method}'. "
                "El IC no puede calcularse hasta completar al menos 30 trades."
            )
        elif ic_live and ic_live.get("ic_value") is not None:
            ic_val = ic_live["ic_value"]
            n_obs = ic_live.get("n_observations", 0)
            if abs(ic_val) < 0.05:
                warnings.append(
                    f"[ic_tracker] IC = {ic_val:.4f} para '{method}' con n={n_obs}. "
                    "Por debajo del umbral mínimo (IC >= 0.05). Señal sin edge confirmado."
                )

        return {
            "academic_support_score": support_score,
            "sources_found": len(found),
            "sources_used": len(ranked),
            "warnings": warnings,
            "key_insights": key_insights,
            "recommendations": recommendations,
            "ic_live_data": ic_live,
            "query": {
                "method": method,
                "timeframe": timeframe,
                "topic": topic,
            },
        }

    # ── Búsqueda libre ────────────────────────────────────────────────────────

    def search(self, query: str, max_results: int = 10) -> list[dict[str, Any]]:
        """Búsqueda full-text sobre título, key_findings, atlas_utility, tags y subtopics.

        Tokeniza la query en palabras y rankea por número de coincidencias.
        Retorna fuentes ordenadas de mayor a menor relevancia.
        """
        if not query or not str(query).strip():
            return []

        tokens = [t.lower() for t in str(query).strip().split() if len(t) >= 2]
        if not tokens:
            return []

        scored: list[tuple[int, dict[str, Any]]] = []
        for src in self._sources:
            # Construir texto indexable para este source
            text_parts = [
                str(src.get("title", "")),
                str(src.get("key_findings", "")),
                str(src.get("atlas_utility", "")),
                str(src.get("topic", "")),
                str(src.get("method", "")),
                " ".join(str(t) for t in (src.get("tags") or [])),
                " ".join(str(s) for s in (src.get("subtopics") or [])),
                " ".join(str(a) for a in (src.get("authors") or [])),
            ]
            haystack = " ".join(text_parts).lower()
            hits = sum(1 for tok in tokens if tok in haystack)
            if hits > 0:
                scored.append((hits, src))

        scored.sort(key=lambda x: x[0], reverse=True)
        return [src for _, src in scored[:max_results]]

    # ── Listado completo ──────────────────────────────────────────────────────

    def all_sources(self) -> list[dict[str, Any]]:
        """Retorna todas las fuentes del catálogo."""
        return list(self._sources)

    def source_by_id(self, source_id: str) -> dict[str, Any] | None:
        """Retorna una fuente por su ID."""
        return self._by_id.get(str(source_id))

    def summary(self) -> dict[str, Any]:
        """Resumen del estado de la biblioteca."""
        by_category: dict[str, int] = {}
        by_confidence: dict[str, int] = {}
        methods_covered: set[str] = set()
        for src in self._sources:
            cat = str(src.get("category", "?"))
            by_category[cat] = by_category.get(cat, 0) + 1
            conf = str(src.get("confidence_level", "?"))
            by_confidence[conf] = by_confidence.get(conf, 0) + 1
            for m in (src.get("scanner_methods") or []):
                methods_covered.add(str(m))
        return {
            "total_sources": len(self._sources),
            "by_category": by_category,
            "category_labels": CATEGORY_LABELS,
            "by_confidence": by_confidence,
            "scanner_methods_covered": sorted(methods_covered),
            "timeframes_covered": sorted(TIMEFRAME_TO_SOURCES.keys()),
        }


# ── IC live data ──────────────────────────────────────────────────────────────

def _read_ic_live_data(method: str | None) -> dict[str, Any] | None:
    """Lee el estado del ICSignalTracker para el método dado.

    Busca el archivo de estado en la ruta estándar del proyecto.
    Retorna None si no hay datos disponibles o el archivo no existe.
    """
    if not method:
        return None
    try:
        # Ruta del estado del IC tracker
        base = Path(__file__).resolve().parent.parent  # atlas_code_quant/
        state_path = base / "data" / "learning" / "ic_signal_tracker_state.json"
        if not state_path.exists():
            return {"method": method, "n_observations": 0, "ic_value": None, "available": False}
        with open(state_path, "r", encoding="utf-8") as f:
            state = json.load(f)
        signals = state.get("signals") or {}
        method_sigs = [
            s for s in signals.values()
            if str(s.get("method", "")) == str(method) and s.get("outcome_available")
        ]
        if not method_sigs:
            return {"method": method, "n_observations": 0, "ic_value": None, "available": True}
        # Calcular IC promedio de las señales disponibles
        ic_values = [s["ic"] for s in method_sigs if s.get("ic") is not None]
        ic_avg = round(sum(ic_values) / len(ic_values), 4) if ic_values else None
        t_stats = [s["t_stat"] for s in method_sigs if s.get("t_stat") is not None]
        t_avg = round(sum(t_stats) / len(t_stats), 3) if t_stats else None
        return {
            "method": method,
            "n_observations": len(method_sigs),
            "ic_value": ic_avg,
            "t_stat": t_avg,
            "ic_valid": ic_avg is not None and abs(ic_avg) >= 0.05,
            "t_stat_valid": t_avg is not None and abs(t_avg) >= 2.0,
            "available": True,
        }
    except Exception:
        return None


# ── Recomendaciones operativas ────────────────────────────────────────────────

def _build_recommendations(
    method: str | None,
    timeframe: str | None,
    sources: list[dict[str, Any]],
) -> list[str]:
    """Genera recomendaciones operativas derivadas de la literatura relevante."""
    recs: list[str] = []
    tf_norm = _TIMEFRAME_ALIASES.get(str(timeframe or "").lower(), str(timeframe or ""))
    tag_pool: set[str] = set()
    for src in sources:
        tag_pool.update(src.get("tags") or [])

    # Recomendaciones por método
    if method == "trend_ema_stack":
        recs.append(
            "Momentum temporal validado (Moskowitz 2012): operar solo cuando precio > EMA estructural. "
            "El IC de esta señal debe superar 0.05 con t-stat >= 2 antes de escalar."
        )
    elif method == "breakout_donchian":
        recs.append(
            "Breakout: confirmar con volumen > media (microestructura). "
            "La ruptura sin volumen es frecuentemente un false breakout (O'Hara 1995)."
        )
    elif method == "rsi_pullback_trend":
        recs.append(
            "Pullback en tendencia: la reversión de corto plazo existe (Jegadeesh 1990) pero es frágil. "
            "Usar solo con filtro de tendencia dominante activo. Limitar holding a 2-5 días."
        )
    elif method == "relative_strength_overlay":
        recs.append(
            "Momentum cross-seccional validado (Jegadeesh & Titman 1993) en horizontes 3-12 meses. "
            "Priorizar top quintil de fuerza relativa del universo activo."
        )
    elif method == "ml_directional":
        recs.append(
            "Modelo ML: verificar Deflated Sharpe > 0 y n_trades OOS suficiente (Lopez de Prado 2018). "
            "No usar como señal primaria sin validación OOS >= 3 meses."
        )
    elif method == "order_flow_proxy":
        recs.append(
            "Order flow: CVD y desequilibrio bid/ask son proxies válidos de información asimétrica "
            "(O'Hara 1995). Útil como filtro de entrada, no como señal standalone."
        )

    # Recomendaciones por temporalidad
    if tf_norm == "intraday":
        recs.append(
            "Intraday: la microestructura domina la señal (O'Hara 1995). "
            "Verificar spread y liquidez antes de entrar. CVD es el indicador más informativo."
        )
    elif tf_norm in ("short", "swing"):
        recs.append(
            "Swing (1-60 días): zona de momentum. Lo & MacKinlay (1988) valida autocorrelación positiva. "
            "Exit governance activo: revisar posición cada 5-10 días vs. trailing stop."
        )
    elif tf_norm == "long":
        recs.append(
            "Largo plazo: precaución — DeBondt & Thaler (1985) documenta reversión después de 12 meses. "
            "En posiciones > 6 meses, evaluar si el momentum original sigue vigente."
        )

    # Recomendaciones por tags detectados
    if "IC_validation" in tag_pool or "signal_quality" in tag_pool:
        recs.append(
            "Señal con soporte IC: validar IC > 0.05 con n >= 30 observaciones (Grinold & Kahn 2000) "
            "antes de declarar edge operativo. El ICSignalTracker mide esto automáticamente."
        )
    if "stop_loss" in tag_pool or "R_multiples" in tag_pool:
        recs.append(
            "Gestión de riesgo: definir stop inicial (Van Tharp 1999) antes de entrar. "
            "Sin stop, el R-múltiple es indefinido y el sistema pierde capacidad de aprendizaje."
        )
    if "options" in tag_pool or "IV_rank" in tag_pool:
        recs.append(
            "Opciones: IV rank > 50% → venta de premium. IV rank < 30% → compra de spreads debit. "
            "DTE óptimo: 21-45 días para captura de theta (Natenberg 1994)."
        )

    if not recs:
        recs.append(
            "Sin recomendación específica para este contexto. "
            "Verificar IC del scanner y respetar stops antes de operar."
        )

    return recs


# ── Singleton ─────────────────────────────────────────────────────────────────

_KB_INSTANCE: KnowledgeBase | None = None


def get_knowledge_base() -> KnowledgeBase:
    """Retorna la instancia singleton de la KnowledgeBase."""
    global _KB_INSTANCE
    if _KB_INSTANCE is None:
        _KB_INSTANCE = KnowledgeBase()
    return _KB_INSTANCE
