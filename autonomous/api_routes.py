"""
Rutas API para ATLAS AUTONOMOUS. Montar en PUSH con prefix /api.
"""
from __future__ import annotations

import logging
from typing import Any

from fastapi import APIRouter, HTTPException

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["Autonomous"])

# Singletons (se inicializan al primer uso)
_health_aggregator = None
_healing_orchestrator = None
_evolution_orchestrator = None
_metrics_aggregator = None
_learning_orchestrator = None


def _get_health():
    global _health_aggregator
    if _health_aggregator is None:
        try:
            from autonomous.health_monitor import HealthAggregator
            _health_aggregator = HealthAggregator()
        except Exception as e:
            logger.warning("HealthAggregator init: %s", e)
    return _health_aggregator

def _get_healing():
    global _healing_orchestrator
    if _healing_orchestrator is None:
        try:
            from autonomous.self_healing import HealingOrchestrator
            _healing_orchestrator = HealingOrchestrator()
        except Exception as e:
            logger.warning("HealingOrchestrator init: %s", e)
    return _healing_orchestrator

def _get_evolution():
    global _evolution_orchestrator
    if _evolution_orchestrator is None:
        try:
            from autonomous.evolution import EvolutionOrchestratorV2
            _evolution_orchestrator = EvolutionOrchestratorV2()
        except Exception as e:
            logger.warning("EvolutionOrchestratorV2 init: %s", e)
    return _evolution_orchestrator

def _get_metrics():
    global _metrics_aggregator
    if _metrics_aggregator is None:
        try:
            from autonomous.telemetry import MetricsAggregator
            _metrics_aggregator = MetricsAggregator()
        except Exception as e:
            logger.warning("MetricsAggregator init: %s", e)
    return _metrics_aggregator

def _get_learning():
    global _learning_orchestrator
    if _learning_orchestrator is None:
        try:
            from autonomous.learning import LearningOrchestrator
            _learning_orchestrator = LearningOrchestrator()
        except Exception as e:
            logger.warning("LearningOrchestrator init: %s", e)
    return _learning_orchestrator


# --- Health ---
@router.get("/health/comprehensive")
def get_health_comprehensive():
    """Salud global: score, components, anomalies, recommendations."""
    agg = _get_health()
    if not agg:
        raise HTTPException(status_code=503, detail="Health monitor not available")
    r = agg.get_global_health()
    return {
        "score": r.score,
        "system_score": r.system_score,
        "services_score": r.services_score,
        "components": r.components,
        "anomalies": [{"severity": a.severity, "metrics_affected": a.metrics_affected} for a in r.anomalies],
        "recommendations": r.recommendations,
        "timestamp": r.timestamp,
    }


@router.get("/health/metrics/system")
def get_health_metrics_system():
    agg = _get_health()
    if not agg:
        raise HTTPException(status_code=503, detail="Health monitor not available")
    from autonomous.health_monitor import SystemMetrics
    sm = SystemMetrics()
    m = sm.get_current_metrics()
    return {"cpu_percent": m.cpu_percent, "ram_percent": m.ram_percent, "disk_usage_percent": m.disk_usage_percent}


@router.get("/health/metrics/services")
def get_health_metrics_services():
    agg = _get_health()
    if not agg:
        raise HTTPException(status_code=503, detail="Health monitor not available")
    from autonomous.health_monitor import ServiceHealth
    sh = ServiceHealth()
    statuses = sh.get_all_services_status()
    return {k: {"online": v.online, "latency_ms": v.latency_ms, "errors_last_5min": v.errors_last_5min} for k, v in statuses.items()}


@router.get("/health/anomalies")
def get_health_anomalies():
    agg = _get_health()
    if not agg:
        return {"anomalies": []}
    trend = agg.get_historical_trend(days=1)
    return {"anomalies": trend[-20:] if trend else []}


# --- Healing ---
@router.post("/healing/trigger")
def post_healing_trigger():
    """Trigger healing manual (sin error concreto)."""
    ho = _get_healing()
    if not ho:
        raise HTTPException(status_code=503, detail="Healing orchestrator not available")
    return {"ok": True, "message": "Use handle_error(error, context) for concrete errors"}


@router.get("/healing/history")
def get_healing_history():
    ho = _get_healing()
    if not ho:
        return {"history": []}
    return {"history": ho.get_history(limit=50)}


@router.get("/healing/strategies")
def get_healing_strategies():
    from autonomous.self_healing.error_classifier import RecoveryStrategy
    return {"strategies": [s.value for s in RecoveryStrategy]}


@router.get("/healing/stats")
def get_healing_stats():
    ho = _get_healing()
    if not ho:
        return {}
    return ho.get_healing_stats()


# --- Evolution ---
@router.post("/evolution/scan")
def post_evolution_scan():
    ev = _get_evolution()
    if not ev:
        raise HTTPException(status_code=503, detail="Evolution orchestrator not available")
    return {"ok": True, "message": "Scan not implemented in stub"}


@router.post("/evolution/update")
def post_evolution_update():
    ev = _get_evolution()
    if not ev:
        raise HTTPException(status_code=503, detail="Evolution orchestrator not available")
    result = ev.execute_full_update_pipeline([])
    return result


@router.get("/evolution/status")
def get_evolution_status():
    ev = _get_evolution()
    if not ev:
        return {"in_progress": False}
    return ev.get_status()


@router.get("/evolution/history")
def get_evolution_history():
    return {"history": []}


# --- Telemetry ---
@router.get("/telemetry/metrics")
def get_telemetry_metrics(source: str | None = None, time_range: int = 3600):
    ma = _get_metrics()
    if not ma:
        return {"metrics": []}
    return {"metrics": ma.get_metrics(source=source, time_range_sec=float(time_range))}


@router.get("/telemetry/dashboards")
def get_telemetry_dashboards():
    return {"dashboards": ["health_overview", "services_status", "self_healing", "evolution"]}


@router.get("/telemetry/dashboard/{name}")
def get_telemetry_dashboard(name: str):
    try:
        from autonomous.telemetry import DashboardEngine
        de = DashboardEngine()
        return de.generate_dashboard_data(name)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


# --- Resilience ---
@router.get("/resilience/queue")
def get_resilience_queue():
    try:
        from autonomous.resilience import PriorityQueue
        pq = PriorityQueue()
        return pq.get_queue_stats()
    except Exception:
        return {}


@router.get("/resilience/throttling")
def get_resilience_throttling():
    try:
        from autonomous.resilience import ResourceThrottler
        rt = ResourceThrottler()
        return rt.get_throttling_stats()
    except Exception:
        return {}


@router.get("/resilience/survival")
def get_resilience_survival():
    try:
        from autonomous.resilience import SurvivalMode
        return {"active": SurvivalMode.is_in_survival()}
    except Exception:
        return {"active": False}


# --- Learning ---
@router.get("/learning/patterns")
def get_learning_patterns():
    lo = _get_learning()
    if not lo:
        return {"patterns": {}}
    from autonomous.learning import PatternAnalyzer
    pa = PatternAnalyzer()
    return pa.analyze_usage_patterns()


@router.get("/learning/insights")
def get_learning_insights():
    lo = _get_learning()
    if not lo:
        return {"insights": []}
    return {"insights": lo.get_learning_insights()}


@router.post("/learning/feedback")
def post_learning_feedback(context: str = "", feedback_type: str = "satisfaction", value: float = 0.5):
    try:
        from autonomous.learning import FeedbackLoop
        fl = FeedbackLoop()
        fl.record_feedback(context, feedback_type, value)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
