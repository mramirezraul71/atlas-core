"""
TracingSystem - Distributed tracing: trace_id por request, spans por componente con duración.
"""
from __future__ import annotations

import logging
import time
import uuid
from collections import defaultdict
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class TracingSystem:
    """
    start_trace(operation_name) → trace_id;
    add_span(trace_id, component, duration, metadata);
    get_trace(trace_id); analyze_slowest_traces(time_range).
    """

    def __init__(self):
        self._traces: dict[str, dict[str, Any]] = {}
        self._spans: dict[str, list[dict]] = defaultdict(list)
        self._max_traces = 500
        self._trace_times: list[tuple[float, str]] = []

    def start_trace(self, operation_name: str = "") -> str:
        """Genera trace_id único."""
        trace_id = str(uuid.uuid4())[:16]
        self._traces[trace_id] = {
            "operation": operation_name,
            "started_at": time.time(),
            "spans": [],
        }
        self._trace_times.append((time.time(), trace_id))
        if len(self._traces) > self._max_traces:
            self._evict_oldest()
        return trace_id

    def _evict_oldest(self) -> None:
        if len(self._trace_times) < 100:
            return
        to_remove = self._trace_times[:50]
        self._trace_times = self._trace_times[50:]
        for _, tid in to_remove:
            self._traces.pop(tid, None)
            self._spans.pop(tid, None)

    def add_span(self, trace_id: str, component: str, duration_sec: float, metadata: dict | None = None) -> None:
        """Añade un span al trace."""
        span = {"component": component, "duration_sec": duration_sec, "metadata": metadata or {}}
        if trace_id in self._traces:
            self._traces[trace_id]["spans"].append(span)
        self._spans[trace_id].append(span)

    def get_trace(self, trace_id: str) -> dict | None:
        """Devuelve el trace completo con todos los spans."""
        t = self._traces.get(trace_id)
        if not t:
            return None
        return {
            **t,
            "trace_id": trace_id,
            "total_duration_sec": sum(s.get("duration_sec", 0) for s in t.get("spans", [])),
        }

    def analyze_slowest_traces(self, time_range_sec: float = 3600) -> list[dict]:
        """Identifica los traces más lentos en el rango."""
        cutoff = time.time() - time_range_sec
        slow = []
        for trace_id, t in list(self._traces.items()):
            if t["started_at"] < cutoff:
                continue
            total = sum(s.get("duration_sec", 0) for s in t.get("spans", []))
            if total > 0:
                slow.append({"trace_id": trace_id, "operation": t.get("operation", ""), "total_sec": total})
        slow.sort(key=lambda x: x["total_sec"], reverse=True)
        return slow[:20]
