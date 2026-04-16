from __future__ import annotations

import argparse
import json
import os
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import requests


def _safe_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _http_json(url: str, headers: dict | None = None) -> dict:
    try:
        r = requests.get(url, timeout=8, headers=headers or {})
        data = r.json() if "application/json" in (r.headers.get("content-type") or "").lower() else {}
        if isinstance(data, dict):
            data["_status_code"] = r.status_code
            return data
        return {"_status_code": r.status_code, "data": data}
    except Exception as exc:
        return {"_error": str(exc)}


def build_message(label: str = "CIERRE_MERCADO") -> str:
    base = os.getenv("ATLAS_QUANT_BASE", "http://127.0.0.1:8795").rstrip("/")
    api_key = (os.getenv("ATLAS_QUANT_API_KEY") or os.getenv("QUANT_API_KEY") or "atlas-quant-local").strip()
    headers = {"x-api-key": api_key}

    health = _http_json(f"{base}/health")
    paper = _http_json(f"{base}/paper/account", headers=headers)
    op_lite = _http_json(f"{base}/operation/status/lite", headers=headers)

    data_dir = Path(r"C:\ATLAS_PUSH\atlas_code_quant\data\operation")
    oc = _safe_json(data_dir / "operation_center_state.json")
    ex = _safe_json(data_dir / "auton_executor_state.json")

    now = datetime.now(ZoneInfo("America/New_York")).strftime("%Y-%m-%d %H:%M:%S %Z")
    last_decision = (oc.get("last_decision") or {}) if isinstance(oc.get("last_decision"), dict) else {}
    last_candidate = (oc.get("last_candidate") or {}) if isinstance(oc.get("last_candidate"), dict) else {}
    last_action = (ex.get("last_action") or {}) if isinstance(ex.get("last_action"), dict) else {}

    open_positions = health.get("open_positions")
    equity = None
    if isinstance(paper.get("data"), dict):
        equity = paper["data"].get("total_equity") or paper["data"].get("equity")
    elif isinstance(op_lite.get("data"), dict):
        eq = op_lite["data"].get("monitor_summary", {}).get("balances", {}).get("total_equity")
        equity = eq

    lines = [
        f"📘 [ATLAS Code-Quant][{label}]",
        f"{now}",
        "",
        f"• Salud Quant: {health.get('status', 'unknown')} (HTTP {health.get('_status_code', '?')})",
        f"• Posiciones abiertas: {open_positions if open_positions is not None else 'n/d'}",
        f"• Equity paper: ${equity:,.2f}" if isinstance(equity, (int, float)) else "• Equity paper: n/d",
        f"• Modo: {oc.get('auton_mode', 'n/d')} | Executor: {oc.get('executor_mode', 'n/d')}",
        f"• Último candidato: {last_candidate.get('symbol', 'n/d')} / {last_candidate.get('strategy_type', 'n/d')} / {last_candidate.get('action', 'n/d')}",
        f"• Última decisión: {last_decision.get('decision', 'n/d')} | allowed={last_decision.get('allowed', 'n/d')}",
        f"• Motivo: {last_decision.get('reason', 'n/d')}",
        f"• Última acción executor: {last_action.get('decision', 'n/d')} ({last_action.get('timestamp', 'n/d')})",
        "",
        "Estado cierre: informe generado automáticamente por ATLAS.",
    ]
    return "\n".join(lines)


def send_message(message: str) -> tuple[bool, str | None]:
    # Reusa el canal robusto existente en scripts/atlas_market_open_supervisor.py
    try:
        from scripts.atlas_market_open_supervisor import _emit_telegram_sync

        ok, err = _emit_telegram_sync(message)
        return bool(ok), err
    except Exception as exc:
        return False, str(exc)


def main() -> int:
    parser = argparse.ArgumentParser(description="Enviar informe de cierre de mercado por Telegram (Code-Quant).")
    parser.add_argument("--label", default="CIERRE_MERCADO")
    parser.add_argument("--print-only", action="store_true")
    args = parser.parse_args()

    message = build_message(label=args.label)
    if args.print_only:
        print(message)
        return 0

    ok, err = send_message(message)
    out = {"ok": ok, "error": err}
    print(json.dumps(out, ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

