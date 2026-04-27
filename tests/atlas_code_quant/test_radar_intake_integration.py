"""F3 — Verificar que api/main.py usa intake Radar (no scanner) en path activo.

Resuelve la ruta relativa al repo en lugar de usar paths Windows hardcoded,
para que los tests corran tanto en la máquina del operador como en CI/cloud.
"""
from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_PATH = REPO_ROOT / "atlas_code_quant" / "api" / "main.py"


def test_main_uses_radar_intake_in_active_path() -> None:
    content = MAIN_PATH.read_text(encoding="utf-8")
    assert "_SCANNER = RadarScannerAdapter(_RADAR_CLIENT)" in content
    assert "fetch_opportunities(" in content
    assert "\"source\": \"radar_intake\"" in content
    assert "_auto_cycle_mark_stage(\"scanner_report\"" in content


def test_main_no_operational_scanner_import() -> None:
    content = MAIN_PATH.read_text(encoding="utf-8")
    assert "from scanner.opportunity_scanner import OpportunityScannerService" not in content
