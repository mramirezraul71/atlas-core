from __future__ import annotations

import argparse
import csv
import json
import sqlite3
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DB_PATH = ROOT / "atlas_code_quant" / "data" / "journal" / "trading_journal.sqlite3"
OUTPUT_DIR = ROOT / "output" / "journal_forensics"


def _parse_dt(value: str | None) -> datetime | None:
    if not value:
        return None
    raw = str(value).replace("Z", "+00:00")
    for candidate in (raw, raw.split(".")[0]):
        try:
            dt = datetime.fromisoformat(candidate)
            if dt.tzinfo is not None:
                dt = dt.astimezone(timezone.utc).replace(tzinfo=None)
            return dt
        except Exception:
            pass
    return None


def _load_rows() -> list[dict[str, object]]:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    rows = [dict(row) for row in conn.execute("select * from trading_journal where status='closed' order by coalesce(exit_time, updated_at, entry_time) asc")]
    conn.close()
    return rows


def _day_of(row: dict[str, object]) -> str:
    for key in ("exit_time", "updated_at", "entry_time"):
        value = row.get(key)
        if value:
            return str(value)[:10]
    return ""


def _duration_seconds(row: dict[str, object]) -> float | None:
    entry_time = _parse_dt(row.get("entry_time"))  # type: ignore[arg-type]
    exit_time = _parse_dt(row.get("exit_time"))  # type: ignore[arg-type]
    if entry_time is None or exit_time is None:
        return None
    return (exit_time - entry_time).total_seconds()


def _is_clean(row: dict[str, object]) -> bool:
    entry_price = row.get("entry_price")
    exit_price = row.get("exit_price")
    duration = _duration_seconds(row)
    if isinstance(entry_price, (int, float)) and float(entry_price) <= 0:
        return False
    if isinstance(exit_price, (int, float)) and float(exit_price) <= 0:
        return False
    if duration is not None and duration <= 5:
        return False
    if _day_of(row) in {"2026-03-29", "2026-03-30"}:
        return False
    return True


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Atlas journal forensic quarantine")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()

    rows = _load_rows()
    burst_0329 = [row for row in rows if _day_of(row) == "2026-03-29"]
    burst_0330 = [row for row in rows if _day_of(row) == "2026-03-30"]
    clean_rows = [row for row in rows if _is_clean(row)]
    clean_ids = {row.get("id") for row in clean_rows}
    quarantine_rows = [row for row in rows if row.get("id") not in clean_ids]

    top_symbols = Counter(str(row.get("symbol") or "") for row in burst_0330).most_common(10)
    duplicate_groups = Counter((str(row.get("symbol") or ""), str(row.get("exit_time") or "")[:19]) for row in burst_0330)
    duplicate_count = sum(1 for _key, count in duplicate_groups.items() if count > 1)
    quality_score = round((len(clean_rows) / max(len(rows), 1)) * 100.0, 2)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    clean_path = args.output_dir / "journal_clean_20260411.csv"
    quarantine_path = args.output_dir / "journal_quarantine_20260411.csv"
    json_path = args.output_dir / "journal_forensics_20260411.json"
    _write_csv(clean_path, clean_rows)
    _write_csv(quarantine_path, quarantine_rows)

    payload = {
        "burst_0329": len(burst_0329),
        "burst_0330": len(burst_0330),
        "top_symbols_0330": top_symbols,
        "duplicate_symbol_timestamp_groups_0330": duplicate_count,
        "total_closed": len(rows),
        "clean_rows": len(clean_rows),
        "quarantine_rows": len(quarantine_rows),
        "quality_score_pct": quality_score,
        "clean_csv": str(clean_path),
        "quarantine_csv": str(quarantine_path),
    }
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
