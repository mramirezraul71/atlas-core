from __future__ import annotations

from pathlib import Path

import pytest

from modules.atlas_radar_kalshi.brain import LLMReading, RadarBrain
from modules.atlas_radar_kalshi.config import RadarSettings


def _minimal_settings(tmp_path: Path, **kwargs: object) -> RadarSettings:
    pem = tmp_path / "k.pem"
    pem.write_text("dummy", encoding="utf-8")
    base = {
        "kalshi_api_key_id": "kid",
        "kalshi_private_key_path": pem,
    }
    base.update(kwargs)
    return RadarSettings(**base)


@pytest.mark.asyncio
async def test_ask_atlas_brain_parses_json_response(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    settings = _minimal_settings(
        tmp_path,
        llm_backend="atlas_brain",
        atlas_brain_fallback_ollama=False,
    )
    brain = RadarBrain(settings=settings)

    class _Resp:
        def raise_for_status(self) -> None:
            return None

        def json(self) -> dict:
            return {
                "ok": True,
                "response": '{"p_yes": 0.72, "confidence": 0.81, "rationale": "test"}',
                "model_used": "ollama:llama3.1:latest",
            }

    class _Client:
        async def __aenter__(self) -> "_Client":
            return self

        async def __aexit__(self, *a: object) -> None:
            return None

        async def post(self, url: str, json: dict | None = None) -> _Resp:
            assert url.endswith("/brain/process")
            return _Resp()

    import modules.atlas_radar_kalshi.brain as brain_mod

    monkeypatch.setattr(brain_mod.httpx, "AsyncClient", lambda **kw: _Client())
    out = await brain._ask_atlas_brain("prompt")
    assert isinstance(out, LLMReading)
    assert abs(out.p_yes - 0.72) < 1e-6
    assert abs(out.confidence - 0.81) < 1e-6
    assert "test" in out.rationale


@pytest.mark.asyncio
async def test_resolve_llm_fallback_to_ollama_on_atlas_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    settings = _minimal_settings(tmp_path, llm_backend="atlas_brain", atlas_brain_fallback_ollama=True)
    brain = RadarBrain(settings=settings)

    async def _boom(_prompt: str) -> LLMReading:
        raise RuntimeError("network")

    async def _ollama_ok(prompt: str) -> LLMReading:
        assert "JSON" in prompt or "Kalshi" in prompt
        return LLMReading(p_yes=0.6, confidence=0.5, rationale="fallback_ok")

    monkeypatch.setattr(brain, "_ask_atlas_brain", _boom)
    monkeypatch.setattr(brain, "_call_ollama_prompt", _ollama_ok)

    from modules.atlas_radar_kalshi.scanner import OrderBookSnapshot

    book = OrderBookSnapshot(
        market_ticker="T1",
        yes_bids=[(48, 10)],
        yes_asks=[(52, 10)],
        no_bids=[(47, 10)],
        no_asks=[(49, 10)],
    )
    import pandas as pd

    hist = pd.DataFrame({"yes_mid": [0.5] * 40})
    r = await brain._resolve_llm_reading("T1", {}, "", book, hist)
    assert r.rationale == "fallback_ok"
