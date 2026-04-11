"""Contrato HTTP: StdResponse.ok y data.ready alineados en /operation/readiness."""
from __future__ import annotations

from unittest.mock import patch

import pytest
from fastapi.testclient import TestClient

pytest.importorskip("fastapi")


def test_operation_readiness_ok_aligns_with_data_ready_true() -> None:
    import atlas_code_quant.api.main as main

    def _fake_fast() -> dict[str, object]:
        return {"ready": True, "reasons_not_ready": []}

    with patch.object(main, "_operation_readiness_payload_fast", _fake_fast):
        with TestClient(main.app) as client:
            r = client.get(
                "/operation/readiness",
                headers={"x-api-key": main.settings.api_key},
            )
    assert r.status_code == 200
    body = r.json()
    assert body["ok"] is True
    assert body["data"]["ready"] is True


def test_operation_readiness_ok_aligns_with_data_ready_false() -> None:
    import atlas_code_quant.api.main as main

    def _fake_fast() -> dict[str, object]:
        return {"ready": False, "reasons_not_ready": ["unit"]}

    with patch.object(main, "_operation_readiness_payload_fast", _fake_fast):
        with TestClient(main.app) as client:
            r = client.get(
                "/operation/readiness",
                headers={"x-api-key": main.settings.api_key},
            )
    assert r.status_code == 200
    body = r.json()
    assert body["ok"] is False
    assert body["data"]["ready"] is False


def test_operation_readiness_401_without_key() -> None:
    import atlas_code_quant.api.main as main

    def _fake_fast() -> dict[str, object]:
        return {"ready": True, "reasons_not_ready": []}

    with patch.object(main, "_operation_readiness_payload_fast", _fake_fast):
        with TestClient(main.app) as client:
            r = client.get("/operation/readiness")
    assert r.status_code == 401
