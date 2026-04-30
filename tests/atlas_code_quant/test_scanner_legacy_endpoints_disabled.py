from __future__ import annotations

import asyncio
import importlib
import json


def _reload_main_with_legacy_disabled(monkeypatch):
    monkeypatch.setenv("ATLAS_LEGACY_SCANNER_ENABLED", "0")
    monkeypatch.setenv("ATLAS_RADAR_INTAKE_ENABLED", "1")
    mod = importlib.import_module("atlas_code_quant.api.main")
    return importlib.reload(mod)


def test_legacy_scanner_endpoints_return_410(monkeypatch) -> None:
    main = _reload_main_with_legacy_disabled(monkeypatch)
    api_key = main.settings.api_key

    async def _call_all():
        r1 = await main.scanner_status(x_api_key=api_key)
        r2 = await main.scanner_report(x_api_key=api_key, activity_limit=5)
        r3 = await main.scanner_universe_search(q="SP", limit=5, x_api_key=api_key)
        return r1, r2, r3

    responses = asyncio.run(_call_all())
    for resp in responses:
        assert getattr(resp, "status_code", None) == 410
        payload = json.loads(resp.body.decode("utf-8"))
        assert payload.get("ok") is False
        data = payload.get("data") or {}
        assert data.get("deprecated") is True
        assert "/api/radar/opportunities" in str(data.get("message") or "")
