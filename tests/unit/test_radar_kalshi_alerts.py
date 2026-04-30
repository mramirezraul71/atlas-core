from __future__ import annotations

from modules.atlas_radar_kalshi.alerts import AlertEngine


def test_alert_cooldown_behavior(tmp_path) -> None:
    eng = AlertEngine(log_dir=str(tmp_path), cooldown_s=999999)
    assert eng.should_send("k1") is True
    assert eng.should_send("k1") is False


async def _run_eval() -> list[dict]:
    eng = AlertEngine(log_dir=".", cooldown_s=0, min_hedge_rate_1h=0.8)
    # sin canales configurados -> no excepción, lista vacía
    out = await eng.evaluate_and_alert(degraded=True, hedge_rate_1h=0.2, total_1h=10)
    return out


def test_alert_evaluate_no_channels() -> None:
    import asyncio

    out = asyncio.run(_run_eval())
    assert isinstance(out, list)

