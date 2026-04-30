"""Notificaciones Telegram para paper trading (envío y seguimiento de fills)."""
from __future__ import annotations

import json
import logging
import threading
import time
import urllib.request
from typing import Any

from api.schemas import OrderRequest

logger = logging.getLogger("quant.paper_trade_telegram")


def _emit_via_repo_script(text: str) -> bool:
    try:
        import sys
        from pathlib import Path

        root = Path(__file__).resolve().parents[2]
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from scripts.atlas_market_open_supervisor import _emit_telegram_sync

        ok, _err = _emit_telegram_sync(text)
        return bool(ok)
    except Exception:
        return False


def _raw_telegram_send(text: str) -> bool:
    try:
        import os

        from modules.humanoid.config.vault import load_vault_env
        from modules.humanoid.notify import _cached_chat_id

        load_vault_env()
        token = (os.getenv("TELEGRAM_BOT_TOKEN") or os.getenv("TELEGRAM_TOKEN") or "").strip()
        chat_id = (_cached_chat_id() or "").strip()
        if not token or not chat_id:
            return False
        body = json.dumps(
            {
                "chat_id": chat_id,
                "text": (text or "")[:3500],
                "disable_web_page_preview": True,
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            f"https://api.telegram.org/bot{token}/sendMessage",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=20) as response:
            payload = json.loads(response.read().decode("utf-8", errors="replace"))
        return bool(payload.get("ok"))
    except Exception:
        return False


def emit_paper_trade_message(text: str) -> None:
    if not (text or "").strip():
        return
    if _emit_via_repo_script(text):
        return
    if _raw_telegram_send(text):
        return
    logger.debug("[paper_trade_telegram] No se pudo enviar Telegram (canal no disponible).")


def _extract_tradier_flat(execution_result: dict[str, Any]) -> dict[str, Any]:
    resp = execution_result.get("response")
    if not isinstance(resp, dict):
        return {}
    tr = resp.get("tradier_response")
    return tr if isinstance(tr, dict) else {}


def _order_id_from_tr(tr: dict[str, Any]) -> str:
    raw = tr.get("raw_response") if isinstance(tr.get("raw_response"), dict) else {}
    for candidate in (tr, raw):
        for key in ("order_id", "id", "broker_order_id"):
            v = candidate.get(key)
            if v not in {None, ""}:
                return str(v)
    return ""


def _filled_qty_from_tr(tr: dict[str, Any]) -> float:
    raw = tr.get("raw_response") if isinstance(tr.get("raw_response"), dict) else {}
    for candidate in (tr, raw):
        q = candidate.get("exec_quantity") or candidate.get("filled_quantity") or candidate.get("filled_qty")
        if q is not None:
            try:
                return float(q)
            except Exception:
                pass
    return float(tr.get("filled_qty") or 0.0)


def _status_from_tr(tr: dict[str, Any]) -> str:
    raw = tr.get("raw_response") if isinstance(tr.get("raw_response"), dict) else {}
    for candidate in (tr, raw):
        s = candidate.get("status")
        if s:
            return str(s).lower()
    return str(tr.get("status") or "unknown").lower()


def notify_paper_submission(order: OrderRequest, execution_result: dict[str, Any]) -> None:
    tr = _extract_tradier_flat(execution_result)
    oid = _order_id_from_tr(tr)
    st = _status_from_tr(tr)
    filled = _filled_qty_from_tr(tr)
    sym = str(order.symbol or "").upper()
    side = str(order.side or "")
    qty = float(order.size or 0.0)
    line1 = (
        f"ATLAS Quant | PAPER ORDEN\n"
        f"{sym} {side} qty={qty}\n"
        f"order_id={oid or 'n/d'} | broker_status={st} | filled={filled}"
    )
    if filled >= max(qty, 1e-9) or st in {"filled", "complete", "completed"}:
        line1 += "\n(Ejecución completa o sin cantidad pendiente.)"
    emit_paper_trade_message(line1)
    if oid and filled + 1e-9 < qty:
        threading.Thread(
            target=_poll_fill_worker,
            kwargs={
                "order_id": oid,
                "symbol": sym,
                "expected_qty": qty,
                "account_scope": str(order.account_scope or "paper"),
            },
            daemon=True,
            name="atlas-paper-fill-poll",
        ).start()


def _poll_fill_worker(
    *,
    order_id: str,
    symbol: str,
    expected_qty: float,
    account_scope: str,
    max_wait_sec: float = 180.0,
    interval_sec: float = 3.0,
) -> None:
    try:
        from execution.tradier_controls import resolve_account_session

        client, session = resolve_account_session(account_scope=account_scope, account_id=None)
        aid = session.account_id
        deadline = time.time() + max_wait_sec
        last_filled = -1.0
        while time.time() <= deadline:
            try:
                orders = client.orders(aid)
            except Exception as exc:
                logger.debug("[paper_fill_poll] orders() error: %s", exc)
                time.sleep(interval_sec)
                continue
            for bo in orders or []:
                if not isinstance(bo, dict):
                    continue
                oid = str(bo.get("id") or bo.get("order_id") or "")
                if oid != order_id:
                    continue
                fq = float(bo.get("exec_quantity") or bo.get("filled_quantity") or 0.0)
                st = str(bo.get("status") or "").lower()
                sym = str(bo.get("symbol") or symbol).upper()
                if fq > last_filled + 1e-9:
                    last_filled = fq
                    emit_paper_trade_message(
                        f"ATLAS Quant | PAPER FILL UPDATE\n{sym} order_id={oid}\n"
                        f"filled_qty={fq} / {expected_qty} | status={st}"
                    )
                if fq + 1e-9 >= expected_qty or st in {"filled", "complete", "completed"}:
                    return
            time.sleep(interval_sec)
    except Exception as exc:
        logger.debug("[paper_fill_poll] abort: %s", exc)
