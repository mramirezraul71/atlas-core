from __future__ import annotations

from pathlib import Path


def test_main_uses_radar_intake_in_active_path() -> None:
    main_path = Path("C:/ATLAS_PUSH/atlas_code_quant/api/main.py")
    content = main_path.read_text(encoding="utf-8")
    assert "_SCANNER = RadarScannerAdapter(_RADAR_CLIENT)" in content
    assert "fetch_opportunities(" in content
    assert "summary\": {\n                \"candidate_count\": len(candidates),\n                \"source\": \"radar_intake\"" in content
    assert "_auto_cycle_mark_stage(\"scanner_report\"" in content


def test_main_no_operational_scanner_import() -> None:
    main_path = Path("C:/ATLAS_PUSH/atlas_code_quant/api/main.py")
    content = main_path.read_text(encoding="utf-8")
    assert "from scanner.opportunity_scanner import OpportunityScannerService" not in content
