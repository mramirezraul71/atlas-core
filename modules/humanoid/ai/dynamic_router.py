"""Dynamic AI Router: monitorea latencia, error rate, calidad; adapta routing; fallback + memoria."""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Tuple

from .telemetry import aggregate_by_provider, get_recent, quality_insufficient_count
from .registry import resolve_model_for_route, get_model_specs
from .models import ModelSpec


def _ollama_available() -> bool:
    try:
        from modules.humanoid.deploy.healthcheck import _check_llm_reachable
        return _check_llm_reachable().get("ok", False)
    except Exception:
        return False


def get_dynamic_weights(route: str) -> Dict[str, float]:
    """Pesos por provider:model según telemetría. Menor error_rate y latencia => mayor peso."""
    agg = aggregate_by_provider()
    weights: Dict[str, float] = {}
    route_specs = [s for s in get_model_specs(_ollama_available()) if s.route == route]
    for spec in route_specs:
        k = f"{spec.provider_id}:{spec.model_key}"
        v = agg.get(k, {})
        req = v.get("requests", 0)
        if req < 3:
            weights[k] = 1.0
            continue
        err = v.get("error_rate", 0)
        lat_avg = v.get("latency_avg_ms", 0)
        quality_issues = quality_insufficient_count(spec.provider_id, spec.model_key)
        penalty = err * 0.5 + min(lat_avg / 10000, 0.3) + min(quality_issues / 10, 0.2)
        weights[k] = max(0.1, 1.0 - penalty)
    return weights


def resolve_with_telemetry(route: str, ollama_ok: bool, prefer_free: bool = True) -> Optional[ModelSpec]:
    """Resuelve modelo para route considerando telemetría. Fallback si falla => registrar en memory."""
    specs = [s for s in get_model_specs(ollama_ok) if s.route == route]
    if not specs:
        return None
    weights = get_dynamic_weights(route)
    scored = []
    for s in specs:
        k = f"{s.provider_id}:{s.model_key}"
        w = weights.get(k, 1.0)
        if prefer_free and s.is_free:
            w *= 1.2
        scored.append((w, s))
    scored.sort(key=lambda x: -x[0])
    return scored[0][1] if scored else specs[0]


def record_failure_and_adjust(provider_id: str, model_key: str) -> None:
    """Registrar fallo en memory para ajuste futuro. Integración con evolution_memory."""
    try:
        from modules.humanoid.evolution_memory import record_model_failure
        record_model_failure(provider_id, model_key)
    except ImportError:
        pass


def route_with_adaptation(
    prompt: str,
    intent_hint: Optional[str] = None,
    modality: str = "text",
    prefer_free: bool = True,
) -> Tuple[Optional[ModelSpec], str]:
    """Decide modelo con telemetría; retorna (spec, route)."""
    from .router import infer_task_profile, _profile_to_route
    profile = infer_task_profile(prompt, intent_hint=intent_hint, modality=modality)
    route = _profile_to_route(profile)
    ollama_ok = _ollama_available()
    spec = resolve_with_telemetry(route, ollama_ok, prefer_free=prefer_free)
    if spec is None:
        spec = resolve_model_for_route(route, ollama_ok, prefer_free=prefer_free)
    return spec, route
