from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "atlas_code_quant") not in sys.path:
    sys.path.insert(0, str(ROOT / "atlas_code_quant"))

from atlas_code_quant.journal.service import rebuild_from_broker_history  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Rebuild Atlas Code Quant journal from broker history")
    parser.add_argument("--start-date", required=True)
    parser.add_argument("--end-date")
    parser.add_argument("--symbols", default="")
    parser.add_argument("--account-scope", default="paper")
    parser.add_argument("--quality-threshold", type=float, default=80.0)
    parser.add_argument("--no-persist", action="store_true")
    args = parser.parse_args()

    symbols = [item.strip().upper() for item in args.symbols.split(",") if item.strip()]
    payload = rebuild_from_broker_history(
        start_date=args.start_date,
        end_date=args.end_date,
        symbols=symbols,
        account_scope=args.account_scope,
        validate_quality=True,
        quality_threshold_pct=args.quality_threshold,
        persist=not args.no_persist,
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
