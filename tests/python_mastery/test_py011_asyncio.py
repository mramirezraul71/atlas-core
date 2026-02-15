from __future__ import annotations

import asyncio
import os

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


@pytest.mark.asyncio
async def test_gather_with_limit_respects_concurrency():
    from training.python_mastery.async_downloader import gather_with_limit

    current = {"n": 0, "max": 0}

    async def worker(x: int) -> int:
        current["n"] += 1
        current["max"] = max(current["max"], current["n"])
        await asyncio.sleep(0.01)
        current["n"] -= 1
        return x * 2

    res = await gather_with_limit(list(range(20)), worker, limit=3, timeout_s=1.0)
    assert res.cancelled is False
    assert res.ok is True
    assert current["max"] <= 3
    assert len(res.results) == 20


@pytest.mark.asyncio
async def test_gather_with_limit_can_be_cancelled():
    from training.python_mastery.async_downloader import gather_with_limit

    async def slow_worker(x: int) -> int:
        await asyncio.sleep(1.0)
        return x

    task = asyncio.create_task(gather_with_limit(list(range(50)), slow_worker, limit=5, timeout_s=None))
    await asyncio.sleep(0.05)
    task.cancel()
    res = await task
    assert res.cancelled is True

