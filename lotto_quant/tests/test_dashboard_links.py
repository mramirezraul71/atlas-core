"""Unit tests for dashboard.links — cross-link URL builders.

These tests intentionally avoid importing `hud.py` so they don't require
Streamlit to be installed in the test environment.
"""

from __future__ import annotations

import pytest


def test_atlas_central_url_default(monkeypatch):
    monkeypatch.delenv("ATLAS_DASHBOARD_URL", raising=False)
    from lotto_quant.dashboard.links import atlas_central_url
    assert atlas_central_url() == "http://127.0.0.1:8791/ui"


def test_atlas_central_url_appends_ui_when_missing(monkeypatch):
    monkeypatch.setenv("ATLAS_DASHBOARD_URL", "http://atlas.local:9000")
    from lotto_quant.dashboard.links import atlas_central_url
    assert atlas_central_url() == "http://atlas.local:9000/ui"


def test_atlas_central_url_keeps_explicit_ui_path(monkeypatch):
    monkeypatch.setenv("ATLAS_DASHBOARD_URL", "https://atlas.example.com/ui")
    from lotto_quant.dashboard.links import atlas_central_url
    assert atlas_central_url() == "https://atlas.example.com/ui"


def test_atlas_central_url_strips_trailing_slash(monkeypatch):
    monkeypatch.setenv("ATLAS_DASHBOARD_URL", "http://atlas.local:9000/")
    from lotto_quant.dashboard.links import atlas_central_url
    assert atlas_central_url() == "http://atlas.local:9000/ui"


def test_atlas_central_url_empty_falls_back_to_default(monkeypatch):
    monkeypatch.setenv("ATLAS_DASHBOARD_URL", "   ")
    from lotto_quant.dashboard.links import atlas_central_url
    assert atlas_central_url() == "http://127.0.0.1:8791/ui"


def test_atlas_lotto_url_default(monkeypatch):
    monkeypatch.delenv("ATLAS_LOTTO_QUANT_URL", raising=False)
    from lotto_quant.dashboard.links import atlas_lotto_url
    assert atlas_lotto_url() == "http://127.0.0.1:8501"


def test_atlas_lotto_url_custom(monkeypatch):
    monkeypatch.setenv("ATLAS_LOTTO_QUANT_URL", "https://lotto.example.com:443/")
    from lotto_quant.dashboard.links import atlas_lotto_url
    assert atlas_lotto_url() == "https://lotto.example.com:443"
