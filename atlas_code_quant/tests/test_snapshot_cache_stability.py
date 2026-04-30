from __future__ import annotations

import asyncio
import threading
import time

from atlas_code_quant.api import main


def test_snapshot_cache_worker_alive_reports_running_thread() -> None:
    cache_key = "test:worker-alive"
    worker = threading.Thread(target=lambda: time.sleep(0.2), daemon=True)
    with main._SNAPSHOT_CACHE_LOCK:
        main._SNAPSHOT_CACHE_WORKERS[cache_key] = worker
    worker.start()
    try:
        assert main._snapshot_cache_worker_alive(cache_key) is True
    finally:
        worker.join(timeout=1.0)
        with main._SNAPSHOT_CACHE_LOCK:
            main._SNAPSHOT_CACHE_WORKERS.pop(cache_key, None)


def test_resolve_cached_payload_returns_warmup_without_duplicate_builder() -> None:
    cache_key = "test:slow-builder"
    calls: list[float] = []
    started = threading.Event()
    release = threading.Event()

    def builder() -> dict[str, object]:
        calls.append(time.perf_counter())
        started.set()
        release.wait(timeout=2.0)
        return {"value": 42}

    async def run() -> dict[str, object]:
        return await main._resolve_cached_payload(
            cache_key=cache_key,
            builder=builder,
            warmup_factory=lambda: {"value": "warmup"},
            timeout_sec=0.1,
            ttl_sec=10.0,
            max_stale_sec=60.0,
        )

    try:
        payload = asyncio.run(run())
        assert started.wait(timeout=1.0) is True
        assert payload["value"] == "warmup"
        assert payload["cache"]["stale"] is True
        assert payload["cache"]["last_error"] == "refresh_pending"
        assert len(calls) == 1
    finally:
        release.set()
        time.sleep(0.1)
        with main._SNAPSHOT_CACHE_LOCK:
            main._SNAPSHOT_CACHE_WORKERS.pop(cache_key, None)
            main._SNAPSHOT_CACHE.pop(cache_key, None)
            main._SNAPSHOT_CACHE_AT.pop(cache_key, None)
            main._SNAPSHOT_CACHE_ERRORS.pop(cache_key, None)


def test_resolve_cached_payload_uses_worker_result_when_ready_fast() -> None:
    cache_key = "test:fast-builder"
    calls: list[float] = []

    def builder() -> dict[str, object]:
        calls.append(time.perf_counter())
        time.sleep(0.05)
        return {"value": 7}

    async def run() -> dict[str, object]:
        return await main._resolve_cached_payload(
            cache_key=cache_key,
            builder=builder,
            warmup_factory=lambda: {"value": "warmup"},
            timeout_sec=0.5,
            ttl_sec=10.0,
            max_stale_sec=60.0,
        )

    try:
        payload = asyncio.run(run())
        assert payload["value"] == 7
        assert payload["cache"]["stale"] is False
        assert len(calls) == 1
    finally:
        with main._SNAPSHOT_CACHE_LOCK:
            main._SNAPSHOT_CACHE_WORKERS.pop(cache_key, None)
            main._SNAPSHOT_CACHE.pop(cache_key, None)
            main._SNAPSHOT_CACHE_AT.pop(cache_key, None)
            main._SNAPSHOT_CACHE_ERRORS.pop(cache_key, None)
