"""Comparador mínimo legacy scanner vs Radar (base shadow-mode F3).

Uso sugerido:
    python -m atlas_code_quant.legacy.compare_scanner_vs_radar --scanner-report scanner.json --radar-opps radar.json
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class DivergenceRow:
    symbol: str
    scanner_present: bool
    radar_present: bool
    scanner_score: float | None
    radar_score: float | None
    score_delta: float | None
    note: str


def _load_json(path: Path) -> dict[str, Any]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{path} no contiene un objeto JSON")
    return raw


def _scanner_rows(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows = {}
    for item in list(payload.get("candidates") or []):
        if not isinstance(item, dict):
            continue
        sym = str(item.get("symbol") or "").strip().upper()
        if sym:
            rows[sym] = item
    return rows


def _radar_rows(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows = {}
    items = payload.get("opportunities") if "opportunities" in payload else payload.get("candidates")
    for item in list(items or []):
        if not isinstance(item, dict):
            continue
        sym = str(item.get("symbol") or "").strip().upper()
        if sym:
            rows[sym] = item
    return rows


def compare_payloads(scanner_payload: dict[str, Any], radar_payload: dict[str, Any]) -> list[DivergenceRow]:
    srows = _scanner_rows(scanner_payload)
    rrows = _radar_rows(radar_payload)
    symbols = sorted(set(srows) | set(rrows))
    out: list[DivergenceRow] = []
    for sym in symbols:
        s = srows.get(sym)
        r = rrows.get(sym)
        s_score = float(s.get("selection_score")) if s and s.get("selection_score") is not None else None
        r_score = float(r.get("score")) if r and r.get("score") is not None else None
        delta = None if s_score is None or r_score is None else round(r_score - s_score, 4)
        note = "aligned"
        if s is None:
            note = "only_radar"
        elif r is None:
            note = "only_scanner"
        elif delta is not None and abs(delta) >= 15:
            note = "score_gap_ge_15"
        out.append(
            DivergenceRow(
                symbol=sym,
                scanner_present=s is not None,
                radar_present=r is not None,
                scanner_score=s_score,
                radar_score=r_score,
                score_delta=delta,
                note=note,
            )
        )
    return out


def _build_cli() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Compara candidatos scanner legacy vs radar F3")
    p.add_argument("--scanner-report", required=True, help="Ruta a JSON del scanner legacy")
    p.add_argument("--radar-opps", required=True, help="Ruta a JSON de oportunidades Radar")
    p.add_argument("--output", default="", help="Ruta opcional para guardar divergencias")
    return p


def main() -> int:
    args = _build_cli().parse_args()
    s_payload = _load_json(Path(args.scanner_report))
    r_payload = _load_json(Path(args.radar_opps))
    rows = compare_payloads(s_payload, r_payload)
    result = {
        "scanner_count": sum(1 for r in rows if r.scanner_present),
        "radar_count": sum(1 for r in rows if r.radar_present),
        "divergences": [r.__dict__ for r in rows if r.note != "aligned"],
        "rows": [r.__dict__ for r in rows],
    }
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
