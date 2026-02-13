"""Run benchmark: execute probes, measure latency/success."""
from __future__ import annotations

import time
from typing import Any, Dict, List

from .probes import get_probes


def run_bench(level: str = "quick") -> Dict[str, Any]:
    """Run probes against AI router. Returns latency, success, recommended mapping."""
    probes = get_probes(level)
    results: List[Dict[str, Any]] = []
    route_map = {"chat": "CHAT", "code": "CODE", "reason": "REASON", "tools": "TOOLS", "vision": "VISION"}

    for probe_type, prompt in probes.items():
        if probe_type == "vision":
            continue
        route = route_map.get(probe_type, "CHAT")
        t0 = time.perf_counter()
        ok = False
        err = ""
        try:
            from modules.humanoid.ai.router import route_and_run
            out, decision, meta = route_and_run(prompt, intent_hint=probe_type, prefer_free=True)
            ok = bool(out and len(str(out).strip()) > 0)
            err = meta.get("used_fallback") and "used_fallback" or ""
        except Exception as e:
            err = str(e)
        ms = int((time.perf_counter() - t0) * 1000)
        results.append({"probe": probe_type, "route": route, "ok": ok, "ms": ms, "error": err})

    # Recommender: use fastest ok model per route
    recommended = {}
    for r in results:
        if r["ok"] and r["ms"] < 15000:
            route = r["route"]
            if route not in recommended or r["ms"] < recommended.get("ms", 99999):
                recommended[route] = {"model": "ollama", "ms": r["ms"]}

    return {
        "ok": True,
        "level": level,
        "results": results,
        "recommended": recommended,
        "summary": f"{sum(1 for r in results if r['ok'])}/{len(results)} passed",
    }
