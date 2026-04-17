from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.learning.trading_implementation_scorecard import (  # noqa: E402
    build_trading_implementation_scorecard,
    persist_trading_implementation_scorecard,
    run_guardrail_pytest,
    write_trading_implementation_scorecard_json,
    write_trading_implementation_scorecard_report,
)


def _default_report_name() -> str:
    return f"reports/atlas_quant_implementation_scorecard_{datetime.now().strftime('%Y%m%d')}.md"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Construye un scorecard medible para verificar cumplimiento e impacto de la implantacion operativa de ATLAS."
    )
    parser.add_argument("--protocol-json", default="reports/trading_self_audit_protocol.json")
    parser.add_argument("--journal-db", default="atlas_code_quant/data/journal/trading_journal.sqlite3")
    parser.add_argument("--brain-state", default="atlas_code_quant/data/operation/quant_brain_bridge_state.json")
    parser.add_argument("--grafana-check-json", default="reports/atlas_grafana_provisioning_check_latest.json")
    parser.add_argument("--json-out", default="reports/atlas_quant_implementation_scorecard.json")
    parser.add_argument("--report-out", default=_default_report_name())
    parser.add_argument("--run-pytest", action="store_true")
    parser.add_argument("--persist-memory", action="store_true")
    args = parser.parse_args()

    pytest_result = None
    if args.run_pytest:
        pytest_result = run_guardrail_pytest(
            ROOT,
            [
                "atlas_code_quant/tests/test_trading_self_audit_protocol.py",
                "atlas_code_quant/tests/test_trading_implementation_scorecard.py",
                "atlas_code_quant/tests/test_position_management_snapshot.py",
                "atlas_code_quant/tests/test_scanner_metric_recalibration.py",
                "atlas_code_quant/tests/test_operation_center_status.py",
                "atlas_code_quant/tests/test_operation_center_autonomous_guards.py",
            ],
        )

    payload = build_trading_implementation_scorecard(
        root=ROOT,
        protocol_path=ROOT / args.protocol_json,
        journal_db_path=ROOT / args.journal_db,
        brain_state_path=ROOT / args.brain_state,
        grafana_check_path=ROOT / args.grafana_check_json,
        pytest_result=pytest_result,
    )
    json_path = write_trading_implementation_scorecard_json(payload, ROOT / args.json_out)
    report_path = write_trading_implementation_scorecard_report(payload, ROOT / args.report_out)
    latest_report_path = write_trading_implementation_scorecard_report(
        payload,
        ROOT / "reports/atlas_quant_implementation_scorecard_latest.md",
    )

    print(f"scorecard_json={json_path}")
    print(f"scorecard_report={report_path}")
    print(f"scorecard_latest_report={latest_report_path}")
    print(f"process_score={payload['headline']['atlas_process_compliance_score']}")
    print(f"usefulness_score={payload['headline']['atlas_implementation_usefulness_score']}")
    print(f"observability_score={payload['metrics']['observability_feedback_score']['value']}")

    if pytest_result:
        print(f"pytest_score={pytest_result['score']}")
        print(f"pytest_summary={pytest_result['summary']}")

    if args.persist_memory:
        result = persist_trading_implementation_scorecard(
            payload,
            report_path=str(report_path.resolve()),
        )
        print(f"bridge_ok={result.get('ok')}")
        print(f"bridge_error={result.get('error')}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
