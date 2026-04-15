from __future__ import annotations

import asyncio

def test_auton_mode_requires_loop(api_main_module) -> None:
    main = api_main_module
    assert main._auton_mode_requires_loop({"auton_mode": "paper_autonomous"}) is True
    assert main._auton_mode_requires_loop({"auton_mode": "paper_supervised"}) is True
    assert main._auton_mode_requires_loop({"auton_mode": "off"}) is False


async def _drain_task(task: asyncio.Task | None) -> None:
    if task is None:
        return
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


def test_ensure_auto_cycle_running_starts_task_when_mode_enabled(monkeypatch, api_main_module) -> None:
    main = api_main_module
    started: dict[str, object] = {}

    async def _fake_loop(interval_sec: int, max_per_cycle: int) -> None:
        started["interval_sec"] = interval_sec
        started["max_per_cycle"] = max_per_cycle
        await asyncio.sleep(3600)

    async def _run() -> None:
        monkeypatch.setattr(main._OPERATION_CENTER, "get_config", lambda: {"auton_mode": "paper_autonomous"})
        monkeypatch.setattr(main, "_auto_cycle_loop", _fake_loop)
        main._AUTO_CYCLE_STATE["running"] = False
        main._AUTO_CYCLE_STATE["loop_interval_sec"] = 77
        main._AUTO_CYCLE_STATE["max_per_cycle"] = 3
        main._auto_cycle_task = None

        created = main._ensure_auto_cycle_running()
        assert created is True
        assert isinstance(main._auto_cycle_task, asyncio.Task)
        await asyncio.sleep(0)
        assert started == {"interval_sec": 77, "max_per_cycle": 3}
        await _drain_task(main._auto_cycle_task)
        main._auto_cycle_task = None

    asyncio.run(_run())
