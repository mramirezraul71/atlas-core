"""Decide provider+model per task; free-first; integrate LLM service and fallback."""
from __future__ import annotations

import os
import time
from typing import Any, Optional, Tuple

from . import policies
from .budgets import can_spend, max_cost_per_task_usd
from .fallback import fallback_chain, propose_paid_upgrade
from .models import RouteDecision, TaskProfile
from .registry import list_providers, resolve_model_for_route
from .telemetry import record as telemetry_record


def _env_bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _ollama_available() -> bool:
    try:
        from modules.humanoid.deploy.healthcheck import _check_llm_reachable
        return _check_llm_reachable().get("ok", False)
    except Exception:
        return False


def infer_task_profile(prompt: str, intent_hint: Optional[str] = None, modality: str = "text") -> TaskProfile:
    """Infer TaskProfile from prompt and hints."""
    from modules.llm.router import HybridRouter
    from modules.llm.config import LLMSettings
    s = LLMSettings()
    router = HybridRouter(
        fast_max_chars=s.FAST_MAX_CHARS,
        chat_max_chars=s.CHAT_MAX_CHARS,
        reason_min_chars=s.REASON_MIN_CHARS,
    )
    route = (intent_hint or "").strip().upper() or None
    decision = router.decide(prompt, forced=route)
    route_name = decision.route

    intent_map = {
        "FAST": "chat", "CHAT": "chat", "CODE": "code", "REASON": "reason",
        "TOOLS": "tools", "VISION": "vision", "ARCHITECT": "reason", "OPTIMIZER": "code",
    }
    intent = intent_map.get(route_name, "chat")
    if intent_hint and intent_hint.lower() in ("ops", "web", "docs"):
        intent = intent_hint.lower()

    n = len(prompt.strip())
    complexity = "high" if n > 4000 else ("med" if n > 1000 else "low")
    latency_need = "fast" if route_name == "FAST" else "normal"
    safety = "high" if "critical" in (intent_hint or "").lower() else "med"

    return TaskProfile(
        intent=intent,
        complexity=complexity,
        latency_need=latency_need,
        modality=modality,
        safety=safety,
    )


def decide_route(profile: TaskProfile, prefer_free: bool = True) -> RouteDecision:
    """Choose provider+model from TaskProfile."""
    ollama_ok = _ollama_available()
    route = _profile_to_route(profile)
    paid_ok = policies.paid_api_allowed()

    spec = resolve_model_for_route(route, ollama_ok, prefer_free=prefer_free)
    if not spec:
        return RouteDecision(
            provider_id="null",
            model_key="null",
            route=route,
            reason="no_model_available",
            is_free=True,
            cost_estimate_usd=0.0,
        )
    if not spec.is_free and not paid_ok:
        return RouteDecision(
            provider_id="null",
            model_key="null",
            route=route,
            reason="paid_not_allowed",
            is_free=True,
            cost_estimate_usd=0.0,
        )
    if not spec.is_free and not can_spend(max_cost_per_task_usd() * 0.5):
        return RouteDecision(
            provider_id="null",
            model_key="null",
            route=route,
            reason="budget_exceeded",
            is_free=True,
            cost_estimate_usd=0.0,
        )
    return RouteDecision(
        provider_id=spec.provider_id,
        model_key=spec.full_key,
        route=route,
        reason=f"route={route}_free_first" if spec.is_free else f"route={route}_paid",
        is_free=spec.is_free,
        cost_estimate_usd=0.0 if spec.is_free else min(max_cost_per_task_usd(), 0.1),
    )


def _profile_to_route(profile: TaskProfile) -> str:
    i = (profile.intent or "chat").lower()
    if i in ("code",): return "CODE"
    if i in ("reason",): return "REASON"
    if i in ("tools",): return "TOOLS"
    if i in ("vision",): return "VISION"
    if i in ("architect",): return "ARCHITECT"
    if i in ("optimizer",): return "OPTIMIZER"
    if profile.modality == "image": return "VISION"
    if profile.latency_need == "fast": return "FAST"
    return "CHAT"


def _call_ollama(model_name: str, prompt: str, system: Optional[str], timeout_s: int) -> Tuple[bool, str, float]:
    """Call existing LLM service with given model. Returns (ok, output, latency_ms)."""
    from modules.llm.schemas import LLMRequest
    from modules.llm.service import LLMService
    t0 = time.perf_counter()
    svc = LLMService()
    req = LLMRequest(
        prompt=prompt,
        system=system,
        route=None,
        model=model_name,
        timeout_override=timeout_s,
    )
    try:
        resp = svc.run(req)
        ms = (time.perf_counter() - t0) * 1000
        return bool(resp.ok), (resp.output or "").strip(), ms
    except Exception as e:
        ms = (time.perf_counter() - t0) * 1000
        return False, str(e), ms


def _call_null(route: str, prompt: str) -> str:
    """Deterministic response when no IA available."""
    from .prompt_templates import system_for_route
    sys = system_for_route(route)
    return f"[NullProvider] Route={route}. System: {sys[:80]}... Escribe tu solicitud y, si Ollama está activo, usa /llm para obtener respuesta de IA."


def route_and_run(
    prompt: str,
    intent_hint: Optional[str] = None,
    modality: str = "text",
    prefer_free: bool = True,
    system_override: Optional[str] = None,
    timeout_s: Optional[int] = None,
) -> Tuple[str, RouteDecision, dict]:
    """
    Infer profile -> decide route -> run (ollama or null). Apply fallback on failure.
    Returns (response_text, decision, meta).
    meta: latency_ms, used_fallback, approval_id (if proposed paid upgrade).
    timeout_s: override for LLM call (default from OLLAMA_TIMEOUT_S env).
    """
    profile = infer_task_profile(prompt, intent_hint=intent_hint, modality=modality)
    decision = decide_route(profile, prefer_free=prefer_free)
    route = decision.route
    ollama_ok = _ollama_available()
    from .prompt_templates import system_for_route
    system = system_override or system_for_route(route)

    meta: dict = {"latency_ms": 0, "used_fallback": False, "approval_id": None}
    last_error = ""

    # Override desde Cerebro (modo manual): el usuario eligió una IA concreta (Ollama o externa)
    try:
        from .brain_state import get_override_full_key
        from .provider_credentials import get_provider_api_key
        from .external_llm import call_external
        full_key = get_override_full_key()
        if full_key and ":" in full_key:
            provider_id = full_key.split(":", 1)[0].strip().lower()
            model_name = full_key.split(":", 1)[1].strip()
            _timeout = timeout_s if timeout_s is not None else int(os.getenv("OLLAMA_TIMEOUT_S", "60") or "60")
            if provider_id == "ollama" and ollama_ok:
                ok, out, ms = _call_ollama(model_name, prompt, system, _timeout)
                meta["latency_ms"] = ms
                telemetry_record("ollama", full_key, ms, ok, 0.0, "" if ok and out else "empty_response")
                if ok and out:
                    decision = RouteDecision(
                        provider_id="ollama",
                        model_key=full_key,
                        route=route,
                        reason="brain_manual_override",
                        is_free=True,
                        cost_estimate_usd=0.0,
                    )
                    return out, decision, meta
                last_error = out or "unknown"
            elif provider_id in ("openai", "anthropic", "gemini", "perplexity") and policies.paid_api_allowed():
                api_key = get_provider_api_key(provider_id)
                if api_key:
                    ok, out, ms = call_external(provider_id, model_name, prompt, system, api_key, _timeout)
                    meta["latency_ms"] = ms
                    telemetry_record(provider_id, full_key, ms, ok, 0.01, "" if ok and out else (out or "")[:200])
                    if ok and out:
                        decision = RouteDecision(
                            provider_id=provider_id,
                            model_key=full_key,
                            route=route,
                            reason="brain_manual_override",
                            is_free=False,
                            cost_estimate_usd=0.01,
                        )
                        return out, decision, meta
                    last_error = out or "unknown"
    except Exception as e:
        last_error = str(e)[:200]

    # Try primary then fallback chain (same route, free first)
    chain = fallback_chain(route, ollama_ok, prefer_free=prefer_free)
    for spec in chain:
        if spec.provider_id == "null":
            t0 = time.perf_counter()
            out = _call_null(route, prompt)
            meta["latency_ms"] = (time.perf_counter() - t0) * 1000
            telemetry_record("null", "null", meta["latency_ms"], True, 0.0, "")
            return out, decision, meta
        if spec.provider_id == "ollama":
            model_name = spec.model_name
            _timeout = timeout_s if timeout_s is not None else int(os.getenv("OLLAMA_TIMEOUT_S", "60") or "60")
            ok, out, ms = _call_ollama(model_name, prompt, system, _timeout)
            meta["latency_ms"] = ms
            telemetry_record("ollama", spec.full_key, ms, ok, 0.0, "" if ok and out else "empty_response")
            if ok and out:
                return out, decision, meta
            last_error = out or "unknown"
            meta["used_fallback"] = True
            continue
        # Paid: would need real API client; for now skip and fall through to null or propose
        if spec.provider_id in ("openai", "anthropic", "gemini", "perplexity"):
            if not policies.paid_api_allowed():
                continue
            # Stub: don't call paid here; propose upgrade
            aid = propose_paid_upgrade("quality_insufficient_or_fallback", decision.cost_estimate_usd or 0.1, route)
            meta["approval_id"] = aid
            break

    # All failed -> null response
    t0 = time.perf_counter()
    out = _call_null(route, prompt)
    meta["latency_ms"] = (time.perf_counter() - t0) * 1000
    if last_error:
        out += f"\n[Último error: {last_error[:200]}]"
    telemetry_record("null", "null", meta["latency_ms"], True, 0.0, "")
    return out, decision, meta
