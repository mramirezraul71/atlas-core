"""Metrics store: counters, latencies, simple in-memory."""
from __future__ import annotations

import time
from collections import defaultdict
from typing import Any, Dict, List


class MetricsStore:
    """In-memory metrics: inc counters, record latencies, snapshot for export."""

    def __init__(self) -> None:
        self._counters: Dict[str, int] = defaultdict(int)
        self._latencies: Dict[str, List[float]] = defaultdict(list)
        self._max_latency_samples = 1000

    def inc(self, name: str, delta: int = 1) -> None:
        self._counters[name] += delta

    def record_latency(self, name: str, ms: float) -> None:
        arr = self._latencies[name]
        arr.append(ms)
        if len(arr) > self._max_latency_samples:
            self._latencies[name] = arr[-self._max_latency_samples :]

    def latency_timer(self, name: str):
        """Context manager to record elapsed ms."""
        class _Timer:
            def __init__(self, store: MetricsStore, n: str):
                self._store = store
                self._name = n
                self._t0 = 0.0
            def __enter__(self):
                self._t0 = time.perf_counter()
                return self
            def __exit__(self, *args):
                self._store.record_latency(self._name, (time.perf_counter() - self._t0) * 1000)
        return _Timer(self, name)

    def snapshot(self) -> Dict[str, Any]:
        """Return current counters and latency stats (count, avg, p95)."""
        out: Dict[str, Any] = {"counters": dict(self._counters)}
        latency_stats: Dict[str, Dict[str, float]] = {}
        for name, arr in self._latencies.items():
            if not arr:
                continue
            s = sorted(arr)
            n = len(s)
            latency_stats[name] = {
                "count": n,
                "avg_ms": sum(s) / n,
                "p95_ms": s[int(n * 0.95)] if n else 0,
            }
        out["latencies"] = latency_stats
        return out

    def reset(self) -> None:
        self._counters.clear()
        self._latencies.clear()
