#!/usr/bin/env python3
"""
Preflight cleanup for Options Fase A: journal phantoms (paper) + Tradier sandbox inventory.

Modes:
  --dry-run (default): read-only; writes markdown report path via --report-md.
  --execute: mutating; requires --journal-purge and/or --sandbox-close.

Safety:
  - Journal: refuses unless SQLite path exists; backs up DB before DELETE.
  - Sandbox: refuses unless Tradier paper base_url contains 'sandbox.tradier.com'.
  - Never targets live accounts (paper scope only).

Reversible:
  - Backup copy of journal DB under output/backups/
  - JSONL of deleted journal row ids + keys under output/backups/
"""
from __future__ import annotations

import argparse
import json
import logging
import re
import shutil
import sqlite3
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
_LOG = logging.getLogger("options_preflight_cleanup")


def _setup_paths() -> None:
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    if str(ROOT / "atlas_code_quant") not in sys.path:
        sys.path.insert(0, str(ROOT / "atlas_code_quant"))


def _journal_db_path() -> Path:
    _setup_paths()
    from config.settings import settings  # noqa: WPS433

    return Path(settings.journal_db_path).resolve()


def _broker_order_ids_empty(raw: str | None) -> bool:
    if raw is None:
        return True
    s = str(raw).strip()
    if s in {"", "{}", "[]", "null"}:
        return True
    try:
        data = json.loads(s)
    except json.JSONDecodeError:
        return True
    if data in ({}, []):
        return True
    if isinstance(data, dict):
        # treat dict with only empty values as empty
        return not any(_nonempty(v) for v in data.values())
    if isinstance(data, list):
        return len(data) == 0 or not any(_nonempty(v) for v in data)
    return False


def _nonempty(v: Any) -> bool:
    if v is None:
        return False
    if isinstance(v, (int, float)):
        return v != 0
    if isinstance(v, str):
        return bool(v.strip())
    if isinstance(v, (list, dict)):
        return bool(v)
    return True


def _is_phantom_row(row: sqlite3.Row, *, require_open: bool) -> bool:
    d = dict(row)
    if str(d.get("account_type") or "").lower() != "paper":
        return False
    if require_open and str(d.get("status") or "").lower() != "open":
        return False
    sid = str(d.get("strategy_id") or "").strip()
    tid = str(d.get("tracker_strategy_id") or "").strip()
    if not sid or not tid:
        return False
    if sid == tid:
        return False
    if not _broker_order_ids_empty(d.get("broker_order_ids_json")):
        return False
    return True


def _is_mismatch_open_paper(row: sqlite3.Row) -> bool:
    d = dict(row)
    if str(d.get("account_type") or "").lower() != "paper":
        return False
    if str(d.get("status") or "").lower() != "open":
        return False
    sid = str(d.get("strategy_id") or "").strip()
    tid = str(d.get("tracker_strategy_id") or "").strip()
    if not sid or not tid or sid == tid:
        return False
    return True


def _journal_sqlite_stats(db_path: Path) -> dict[str, int]:
    conn = sqlite3.connect(str(db_path))
    c = conn.cursor()
    stats: dict[str, int] = {}

    def one(sql: str) -> int:
        row = c.execute(sql).fetchone()
        return int(row[0]) if row and row[0] is not None else 0

    stats["total_rows"] = one("SELECT COUNT(*) FROM trading_journal")
    stats["open_paper"] = one(
        "SELECT COUNT(*) FROM trading_journal WHERE account_type='paper' AND status='open'"
    )
    stats["mismatch_open_paper"] = one(
        "SELECT COUNT(*) FROM trading_journal WHERE account_type='paper' AND status='open' "
        "AND TRIM(strategy_id)!='' AND TRIM(IFNULL(tracker_strategy_id,''))!='' "
        "AND strategy_id!=tracker_strategy_id"
    )
    stats["mismatch_open_paper_nonempty_broker"] = one(
        "SELECT COUNT(*) FROM trading_journal WHERE account_type='paper' AND status='open' "
        "AND TRIM(strategy_id)!='' AND TRIM(IFNULL(tracker_strategy_id,''))!='' "
        "AND strategy_id!=tracker_strategy_id "
        "AND NOT (broker_order_ids_json IS NULL OR TRIM(broker_order_ids_json) IN ('','{}','[]'))"
    )
    conn.close()
    return stats


def _fetch_journal_candidates(
    db_path: Path, *, require_open: bool
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    rows = conn.execute("SELECT * FROM trading_journal").fetchall()
    conn.close()
    all_rows = [dict(r) for r in rows]
    phantoms = [dict(r) for r in rows if _is_phantom_row(r, require_open=require_open)]
    return all_rows, phantoms


def _sandbox_client_and_account() -> tuple[Any, str, str]:
    _setup_paths()
    from backtesting.winning_probability import TradierClient  # noqa: WPS433
    from config.settings import settings  # noqa: WPS433

    base = (settings.tradier_paper_base_url or "").lower()
    if "sandbox.tradier.com" not in base:
        raise RuntimeError(
            f"Tradier paper base_url must be sandbox (got {settings.tradier_paper_base_url!r}). Refusing."
        )
    acct = (settings.tradier_paper_account_id or "").strip()
    if not acct:
        raise RuntimeError("TRADIER_PAPER_ACCOUNT_ID / settings.tradier_paper_account_id empty.")
    client = TradierClient(scope="paper")
    return client, acct, base


def _classify_position(p: dict[str, Any]) -> str:
    sym = str(p.get("symbol") or p.get("underlying_symbol") or "")
    # Heuristic: OCC option symbols are longer and contain digits for date
    if len(sym) > 10 and any(c.isdigit() for c in sym):
        return "option"
    return "equity"


def _group_positions(positions: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    by: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for p in positions:
        cls = _classify_position(p)
        by[cls].append(p)
    return dict(by)


def _occ_underlying_root(occ_symbol: str) -> str:
    """Best-effort OCC root (letters until first digit)."""
    root = []
    for ch in occ_symbol.upper().strip():
        if ch.isalpha():
            root.append(ch)
        elif ch.isdigit():
            break
    s = "".join(root)
    if len(s) >= 1:
        return s
    m = re.match(r"^([A-Z]{1,6})", occ_symbol.upper())
    return m.group(1) if m else "SPY"


def _position_to_close_request(p: dict[str, Any]) -> dict[str, Any] | None:
    """Build kwargs for OrderRequest (dict) — conservative single-leg."""
    sym = str(p.get("symbol") or "").strip()
    if not sym:
        return None
    try:
        qty = float(p.get("quantity") or p.get("qty") or 0)
    except (TypeError, ValueError):
        return None
    if qty == 0:
        return None
    cls = _classify_position(p)
    if cls == "equity":
        if qty > 0:
            side = "sell"
        else:
            side = "buy_to_cover"
        return {
            "symbol": sym.split()[0] if " " in sym else sym,
            "side": side,
            "size": abs(qty),
            "order_type": "market",
            "duration": "day",
            "asset_class": "equity",
            "preview": False,
            "account_scope": "paper",
            "position_effect": "close",
        }
    # single option
    if qty > 0:
        side = "sell_to_close"
    else:
        side = "buy_to_close"
    und = str(p.get("underlying_symbol") or p.get("underlying") or "").strip().upper()
    if not und:
        und = _occ_underlying_root(sym)
    return {
        "symbol": und,
        "side": side,
        "size": abs(qty),
        "order_type": "market",
        "duration": "day",
        "asset_class": "option",
        "option_symbol": sym,
        "preview": False,
        "account_scope": "paper",
        "position_effect": "close",
    }


def run_sandbox_scan() -> dict[str, Any]:
    client, acct, base = _sandbox_client_and_account()
    positions = client.positions(acct)
    grouped = _group_positions(positions)
    return {
        "account_id": acct,
        "base_url": base,
        "position_count": len(positions),
        "grouped_counts": {k: len(v) for k, v in grouped.items()},
        "positions_sample": positions[:25],
        "positions_full_json": positions,
    }


def run_journal_purge(
    db_path: Path,
    phantom_ids: list[int],
    *,
    execute: bool,
    backup_dir: Path,
) -> dict[str, Any]:
    backup_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    result: dict[str, Any] = {
        "timestamp_utc": ts,
        "db_path": str(db_path),
        "phantom_ids_count": len(phantom_ids),
        "executed": execute,
    }
    if not phantom_ids:
        return result
    backup_sqlite = backup_dir / f"journal_pre_purge_{ts}.sqlite3"
    manifest = backup_dir / f"journal_purge_manifest_{ts}.jsonl"
    if execute:
        shutil.copy2(db_path, backup_sqlite)
        result["backup_sqlite"] = str(backup_sqlite)
        conn = sqlite3.connect(str(db_path))
        try:
            cur = conn.cursor()
            lines: list[str] = []
            for pid in phantom_ids:
                r = cur.execute(
                    "SELECT id, journal_key, strategy_id, tracker_strategy_id, symbol, status, entry_time FROM trading_journal WHERE id=?",
                    (pid,),
                ).fetchone()
                if r:
                    lines.append(
                        json.dumps(
                            {
                                "id": r[0],
                                "journal_key": r[1],
                                "strategy_id": r[2],
                                "tracker_strategy_id": r[3],
                                "symbol": r[4],
                                "status": r[5],
                                "entry_time": str(r[6]),
                            },
                            ensure_ascii=False,
                        )
                    )
            manifest.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
            result["manifest_jsonl"] = str(manifest)
            q_marks = ",".join("?" * len(phantom_ids))
            cur.execute(f"DELETE FROM trading_journal WHERE id IN ({q_marks})", phantom_ids)
            conn.commit()
            result["deleted_rows"] = len(phantom_ids)
        finally:
            conn.close()
    return result


def run_sandbox_close(
    positions: list[dict[str, Any]],
    *,
    execute: bool,
    max_closes: int,
    equity_only: bool,
) -> dict[str, Any]:
    from atlas_code_quant.api.schemas import OrderRequest  # noqa: WPS433
    from atlas_code_quant.execution.tradier_execution import route_order_to_tradier  # noqa: WPS433

    out: dict[str, Any] = {"attempted": 0, "ok": 0, "errors": [], "executed": execute}
    for i, p in enumerate(positions):
        if i >= max_closes:
            out["truncated_at"] = max_closes
            break
        if equity_only and _classify_position(p) != "equity":
            continue
        raw = _position_to_close_request(p)
        if not raw:
            continue
        out["attempted"] += 1
        if not execute:
            continue
        try:
            body = OrderRequest.model_validate(raw)
            route_order_to_tradier(body)
            out["ok"] += 1
        except Exception as exc:
            out["errors"].append({"symbol": p.get("symbol"), "error": str(exc)[:500]})
    return out


def _write_md(path: Path, title: str, sections: list[tuple[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"# {title}", "", f"_Generated (UTC): {datetime.now(timezone.utc).isoformat()}_", ""]
    for h, body in sections:
        lines.append(f"## {h}")
        lines.append("")
        lines.append(body.rstrip())
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="Options preflight: journal phantoms + sandbox scan/close")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Alias for default read-only mode (no --execute). Optional for clarity.",
    )
    parser.add_argument("--execute", action="store_true", help="Allow mutations (journal and/or sandbox)")
    parser.add_argument("--journal-purge", action="store_true", help="With --execute: DELETE phantom journal rows")
    parser.add_argument("--sandbox-close", action="store_true", help="With --execute: submit closing orders")
    parser.add_argument(
        "--phantom-include-closed",
        action="store_true",
        help="Treat closed rows too as phantom candidates (default: open-only)",
    )
    parser.add_argument(
        "--journal-purge-tier",
        choices=("strict", "mismatch_open_paper"),
        default="strict",
        help=(
            "strict: paper+open+strategy_id!=tracker_strategy_id+empty broker ids (audit-aligned). "
            "mismatch_open_paper: all open paper rows with id mismatch (destructive; aligns journal reset with sandbox wipe)."
        ),
    )
    parser.add_argument(
        "--i-understand-destructive-journal",
        action="store_true",
        help="Required with --journal-purge-tier mismatch_open_paper",
    )
    parser.add_argument("--max-sandbox-closes", type=int, default=250)
    parser.add_argument(
        "--sandbox-equity-only",
        action="store_true",
        help="With --sandbox-close: only submit equity closes (safer if option roots are ambiguous)",
    )
    parser.add_argument("--report-md", type=Path, default=ROOT / "output" / "options_preflight_cleanup_report.md")
    parser.add_argument("--result-md", type=Path, default=ROOT / "output" / "options_preflight_cleanup_result.md")
    args = parser.parse_args()

    execute = bool(args.execute)
    if execute and not args.journal_purge and not args.sandbox_close:
        print("ERROR: --execute requires --journal-purge and/or --sandbox-close", file=sys.stderr)
        return 2

    if execute and args.journal_purge and args.journal_purge_tier == "mismatch_open_paper":
        if not args.i_understand_destructive_journal:
            print(
                "ERROR: mismatch_open_paper purge requires --i-understand-destructive-journal",
                file=sys.stderr,
            )
            return 2

    db_path = _journal_db_path()
    require_open = not bool(args.phantom_include_closed)
    journal_all: list[dict[str, Any]] = []
    phantoms: list[dict[str, Any]] = []
    sqlite_stats: dict[str, int] = {}
    if db_path.is_file():
        journal_all, phantoms = _fetch_journal_candidates(db_path, require_open=require_open)
        sqlite_stats = _journal_sqlite_stats(db_path)
    else:
        _LOG.warning("Journal DB missing: %s", db_path)

    sandbox_info: dict[str, Any] | None = None
    sandbox_err: str | None = None
    try:
        sandbox_info = run_sandbox_scan()
    except Exception as exc:
        sandbox_err = str(exc)
        _LOG.warning("Sandbox scan skipped: %s", exc)

    phantom_criterion = (
        "paper account_type; "
        + ("status=open; " if require_open else "")
        + "strategy_id and tracker_strategy_id both non-empty and different; "
        + "broker_order_ids_json empty ({}, [], or no meaningful ids)."
    )
    tier_note = ""
    if sqlite_stats:
        tier_note = (
            f"\n- SQL cross-check: open paper **{sqlite_stats.get('open_paper', '?')}**; "
            f"id mismatch (open paper) **{sqlite_stats.get('mismatch_open_paper', '?')}**; "
            f"of those, non-empty broker_order_ids **{sqlite_stats.get('mismatch_open_paper_nonempty_broker', '?')}**.\n"
            "- **Tier strict (default purge)**: mismatch + empty broker ids (audit-style phantoms).\n"
            "- **Tier mismatch_open_paper** (optional, destructive): all open paper with strategy_id≠tracker_strategy_id; "
            "requires `--journal-purge-tier mismatch_open_paper --i-understand-destructive-journal`.\n"
        )

    journal_summary = (
        f"- DB path: `{db_path}`\n"
        f"- Total journal rows: **{len(journal_all)}**\n"
        f"- Phantom candidates (strict): **{len(phantoms)}**\n"
        f"- Criterion (strict): {phantom_criterion}\n"
        f"{tier_note}"
    )
    if phantoms:
        ids = [int(p["id"]) for p in phantoms if p.get("id") is not None]
        journal_summary += f"- Phantom id range: `{min(ids)}` … `{max(ids)}`\n"
        symbols = defaultdict(int)
        for p in phantoms:
            symbols[str(p.get("symbol") or "")] += 1
        top = sorted(symbols.items(), key=lambda x: -x[1])[:15]
        journal_summary += "- Top symbols among phantoms:\n" + "\n".join(f"  - `{s}`: {c}" for s, c in top if s)

    sandbox_summary = "Sandbox scan **not available**."
    if sandbox_info:
        sandbox_summary = (
            f"- Account: `{sandbox_info['account_id']}`\n"
            f"- Base URL: `{sandbox_info['base_url']}`\n"
            f"- Open positions (API): **{sandbox_info['position_count']}**\n"
            f"- Grouped (heuristic): `{sandbox_info['grouped_counts']}`\n"
            "- Grouping: by OCC-style symbol length/digits → **option** vs **equity**.\n"
            "- Expected cleanup impact: closing submits market **close** orders per leg (execute mode); "
            "multi-leg combos may need manual follow-up if API rejects.\n"
        )
    elif sandbox_err:
        sandbox_summary = f"Sandbox scan failed: `{sandbox_err}`"

    impact = (
        "Removing phantom journal rows reduces false reconciliation targets for `sync_scope` "
        "and learning hooks. Closing sandbox positions frees capital and removes stale risk; "
        "residual risk: partial fills, rejected multileg closes."
    )

    inspected = [
        f"`{db_path}` (journal SQLite)",
        "`atlas_code_quant/journal/models.py` (schema reference)",
        "`atlas_code_quant/journal/db.py` (engine reference)",
        "`atlas_code_quant/config/settings.py` (journal_db_path, Tradier paper URLs)",
        "`reports/audit_phantom_journal_20260416.json` (historical phantom sample / criteria alignment)",
        "`scripts/atlas_journal_forensics.py` (existing closed-row forensics — not used for phantoms)",
        "`scripts/journal_rebuild.py` (broker rebuild — not invoked here)",
    ]
    inspected_md = "\n".join(f"- {line}" for line in inspected)

    sections = [
        ("Archivos inspeccionados (referencia)", inspected_md),
        ("Resumen", "Dry-run inventory for Options Fase A preflight. **No mutations in this file.**"),
        ("Journal", journal_summary),
        ("Tradier sandbox", sandbox_summary),
        ("Impacto esperado", impact),
        (
            "Comandos sugeridos",
            "```powershell\n"
            f"cd {ROOT}\n"
            "# Dry-run (this report):\n"
            "python scripts/options_preflight_cleanup.py --dry-run\n"
            "# Execute journal purge only:\n"
            "python scripts/options_preflight_cleanup.py --execute --journal-purge\n"
            "# Execute sandbox closes (after review):\n"
            "python scripts/options_preflight_cleanup.py --execute --sandbox-close\n"
            "```",
        ),
    ]
    if not execute:
        _write_md(args.report_md, "Options preflight cleanup — dry-run report", sections)
        _LOG.info("Wrote report: %s", args.report_md)
        return 0

    # --- execute path ---
    cmdline = " ".join(sys.argv)
    result_sections: list[tuple[str, str]] = [
        ("Resumen", f"Execute run at `{datetime.now(timezone.utc).isoformat()}` UTC."),
        ("Comando", f"`{cmdline}`"),
    ]
    backup_dir = ROOT / "output" / "backups"

    if args.journal_purge:
        if not db_path.is_file():
            result_sections.append(("Journal purge", "SKIPPED: DB not found."))
        else:
            if args.journal_purge_tier == "mismatch_open_paper":
                conn = sqlite3.connect(str(db_path))
                conn.row_factory = sqlite3.Row
                rows = conn.execute("SELECT * FROM trading_journal").fetchall()
                conn.close()
                purge_rows = [dict(r) for r in rows if _is_mismatch_open_paper(r)]
                ids = [int(p["id"]) for p in purge_rows if p.get("id") is not None]
                pre_ph = len(purge_rows)
            else:
                ids = [int(p["id"]) for p in phantoms if p.get("id") is not None]
                pre_ph = len(phantoms)
            purge_result = run_journal_purge(db_path, ids, execute=True, backup_dir=backup_dir)
            _, ph_after = _fetch_journal_candidates(db_path, require_open=require_open)
            purge_result["phantom_count_before"] = pre_ph
            purge_result["phantom_count_after"] = len(ph_after)
            result_sections.append(
                (
                    "Journal purge",
                    "```json\n" + json.dumps(purge_result, indent=2, ensure_ascii=False) + "\n```",
                )
            )

    if args.sandbox_close:
        if sandbox_err or not sandbox_info:
            result_sections.append(("Sandbox close", f"SKIPPED: {sandbox_err or 'no scan data'}"))
        else:
            positions = sandbox_info.get("positions_full_json") or []
            pre_n = len(positions)
            close_result = run_sandbox_close(
                positions,
                execute=True,
                max_closes=args.max_sandbox_closes,
                equity_only=bool(args.sandbox_equity_only),
            )
            close_result["positions_before"] = pre_n
            try:
                client2, acct2, _ = _sandbox_client_and_account()
                post_pos = client2.positions(acct2)
                close_result["positions_after"] = len(post_pos)
                try:
                    orders = client2.orders(acct2)
                    close_result["orders_recent_total"] = len(orders)
                    close_result["orders_pending_approx"] = sum(
                        1
                        for o in orders
                        if str(o.get("status") or "").lower() in {"pending", "open", "submitted", "partially_filled"}
                    )
                except Exception as exc_o:
                    close_result["orders_probe_error"] = str(exc_o)
            except Exception as exc:
                close_result["positions_after_error"] = str(exc)
            result_sections.append(
                (
                    "Sandbox close",
                    "```json\n" + json.dumps(close_result, indent=2, ensure_ascii=False) + "\n```",
                )
            )

    _write_md(args.result_md, "Options preflight cleanup — execute result", result_sections)
    _LOG.info("Wrote result: %s", args.result_md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
