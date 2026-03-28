from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from atlas_code_quant.learning.trading_self_audit_protocol import (  # noqa: E402
    persist_trading_self_audit_note,
    write_trading_self_audit_protocol,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Registra en memoria/bitacora el protocolo logico abierto de auto-auditoria del proceso de trading."
    )
    parser.add_argument(
        "--report-path",
        default="reports/atlas_quant_post_trade_learning_audit_20260328.md",
        help="Ruta del informe que se quiere enlazar en la nota de memoria.",
    )
    parser.add_argument(
        "--json-out",
        default="reports/trading_self_audit_protocol.json",
        help="Ruta donde persistir una copia JSON versionada del protocolo.",
    )
    args = parser.parse_args()

    json_path = write_trading_self_audit_protocol(ROOT / args.json_out)
    result = persist_trading_self_audit_note(
        report_path=str((ROOT / args.report_path).resolve()),
    )
    print(f"protocol_json={json_path}")
    print(f"bridge_ok={result.get('ok')}")
    print(f"bridge_error={result.get('error')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
