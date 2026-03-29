#!/usr/bin/env python
"""autonomous_loop.py — Loop autonomo externo ATLAS-Quant v4.

v4 (signal_score compuesto):
  - SignalComponents proxy desde datos del scanner (win_rate, strength, alignment, mtf)
  - final_signal_score = 0.30*motif + 0.30*TIN + 0.20*MTF + 0.20*regime
  - Gate configurable: --min-signal-score 0.55 (default)
  - Log correcto: SCANNER -> SIGNAL score -> VISION conf -> SUBMIT

v3 (vision):
  - Validacion visual pre-submit via Insta360 / desktop OCR
  - --no-vision-gate / --min-visual-conf

v2:
  - Gate de horario de mercado (9:30-15:45 ET solo)
  - Deduplicacion / cooldown 30min por simbolo

Uso:
    python scripts/autonomous_loop.py
    python scripts/autonomous_loop.py --min-signal-score 0.60   # gate estricto
    python scripts/autonomous_loop.py --min-signal-score 0.0    # solo log, sin bloqueo
    python scripts/autonomous_loop.py --no-vision-gate
"""
from __future__ import annotations

import argparse
import json
import logging
import logging.handlers
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone, timedelta
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

# ── Vision (opcional — no bloquea si el modulo no esta disponible) ────────────
_VISION_AVAILABLE = False
_vision_pipeline = None

def _init_vision() -> None:
    global _VISION_AVAILABLE, _vision_pipeline
    try:
        _ROOT_PY = Path(__file__).resolve().parents[1]
        if str(_ROOT_PY) not in sys.path:
            sys.path.insert(0, str(_ROOT_PY))
        from atlas_code_quant.vision.visual_pipeline import VisualPipeline
        _vision_pipeline = VisualPipeline.get_instance()
        src = _vision_pipeline._capture.source_available()
        _VISION_AVAILABLE = src != "none"
        logger_placeholder = logging.getLogger("atlas.execution.live_loop")
        logger_placeholder.info("Vision pipeline OK — fuente: %s", src)
    except Exception as exc:
        logging.getLogger("atlas.execution.live_loop").warning(
            "Vision no disponible: %s", exc
        )


# ── Signal score gate ─────────────────────────────────────────────────────────
# Mapeo scanner → SignalComponents (proxy cuando LiveLoop no esta corriendo)
#
# Scanner devuelve:
#   local_win_rate_pct  [0-100]  → tin_score  (probabilidad historica de exito)
#   signal_strength_pct [0-100]  → regime_confidence (fuerza de tendencia)
#   alignment_score     [0-100]  → mtf_coherence (alineacion multi-TF)
#   direction + selection_score  → motif_edge (sesgo direccional)
#
# formula: final_score = 0.30*motif + 0.30*tin + 0.20*mtf + 0.20*regime

def _compute_signal_score_proxy(cand: dict) -> tuple[float, dict]:
    """Calcula final_signal_score usando datos del candidato del scanner."""
    sel_score  = float(cand.get("selection_score") or 75.0)
    win_rate   = float(cand.get("local_win_rate_pct") or 65.0)
    strength   = float(cand.get("signal_strength_pct") or 50.0)
    alignment  = float(cand.get("alignment_score") or 50.0)
    direction  = str(cand.get("direction") or "neutral").lower()
    conf_pct   = float((cand.get("confirmation") or {}).get("confidence_pct") or alignment)
    order_flow = float((cand.get("order_flow") or {}).get("score_pct") or 50.0)

    # motif_edge: refleja el sesgo direccional + fuerza del setup
    # selection_score [75-100] → [0.55-1.0], ajustado por direccion
    base_edge = max(0.0, min(1.0, (sel_score - 50.0) / 50.0))
    if direction in {"bajista", "bearish", "short", "down", "sell"}:
        motif_edge = 1.0 - base_edge   # invert: bajista fuerte = bajo edge long
    else:
        motif_edge = base_edge

    # tin_score: win_rate historica [0-100] → [0-1]
    tin_score = max(0.0, min(1.0, win_rate / 100.0))

    # mtf_coherence: alignment + higher_tf confirmation [0-100] → [0-1]
    mtf_coh = max(0.0, min(1.0, (alignment * 0.6 + conf_pct * 0.4) / 100.0))

    # regime_confidence: strength de la tendencia [0-100] → [0-1]
    regime_conf = max(0.0, min(1.0, strength / 100.0))

    # formula final_signal_score (misma que SignalGenerator)
    motif_n = max(0.0, min(1.0, (motif_edge + 1.0) / 2.0))  # normalizar [-1,1]→[0,1]
    score = (0.30 * motif_n + 0.30 * tin_score
             + 0.20 * mtf_coh + 0.20 * regime_conf)
    score = round(max(0.0, min(1.0, score)), 4)

    # tier
    if score >= 0.75:
        tier = "FULL"
    elif score >= 0.65:
        tier = "NORMAL"
    elif score >= 0.55:
        tier = "SMALL"
    else:
        tier = "SKIP"

    components = {
        "motif_edge": round(motif_edge, 3),
        "tin_score": round(tin_score, 3),
        "mtf_coherence": round(mtf_coh, 3),
        "regime_confidence": round(regime_conf, 3),
        "signal_score": score,
        "tier": tier,
    }
    return score, components


def _paper_record_fill(
    base: str,
    api_key: str,
    symbol: str,
    side: str,
    qty: int,
    price: float,
    strategy: str = "",
    signal_score: float = 0.0,
    score_tier: str = "",
    option_strategy: str = "",
) -> None:
    """Registra un fill en el PaperBroker local via REST (fuego y olvida)."""
    if not price or price <= 0:
        return
    try:
        body = {
            "symbol": symbol,
            "side": side,
            "qty": qty,
            "price": price,
            "strategy": strategy,
            "signal_score": round(signal_score, 4),
            "score_tier": score_tier,
            "option_strategy": option_strategy,
        }
        _api_call(base, "/paper/fill", method="POST", body=body,
                  timeout=5, api_key=api_key)
    except Exception as exc:
        logging.getLogger("atlas.loop.paper").warning(
            "paper_record_fill error: %s", exc
        )


def _signal_gate(cand: dict, min_score: float) -> tuple[bool, float, str]:
    """Gate de signal_score compuesto.

    Returns:
        (passed, score, log_line)
    """
    score, comp = _compute_signal_score_proxy(cand)
    log_line = (
        f"score={score:.3f} tier={comp['tier']} "
        f"(motif={comp['motif_edge']:.2f} tin={comp['tin_score']:.2f} "
        f"mtf={comp['mtf_coherence']:.2f} regime={comp['regime_confidence']:.2f})"
    )
    passed = score >= min_score if min_score > 0 else True
    return passed, score, log_line


def _visual_check(direction: str, api_price: float, min_conf: float) -> tuple[bool, str]:
    """Valida la senal contra el estado visual del grafico.

    Returns:
        (ok, reason) — ok=True si pasa, False si se debe bloquear
    """
    if not _VISION_AVAILABLE or _vision_pipeline is None:
        return True, "vision_not_available"
    try:
        from atlas_code_quant.strategy.visual_triggers import VisualTriggerValidator
        ocr = _vision_pipeline.analyze(max_age_sec=60.0)
        signal_type = "BUY" if direction in {"long", "alcista", "bull", "up"} else "SELL"
        vtv = VisualTriggerValidator()
        val = vtv.validate(
            signal_type=signal_type,
            api_price=api_price,
            ocr_result=ocr,
        )
        passed = val.overall_confidence >= min_conf if min_conf > 0 else True
        reason = (
            f"visual_ok conf={val.overall_confidence:.2f} color={ocr.chart_color} "
            f"pattern={ocr.pattern_detected}"
        )
        if not passed:
            reason = (
                f"visual_BLOCKED conf={val.overall_confidence:.2f}<{min_conf} "
                f"color={ocr.chart_color} pattern={ocr.pattern_detected}"
            )
        return passed, reason
    except Exception as exc:
        return True, f"visual_error:{exc}"

# ── Logging ───────────────────────────────────────────────────────────────────
_ROOT = Path(__file__).resolve().parents[1]
_LOG_FILE = _ROOT / "logs" / "atlas_live_loop.log"
_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)

_fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S")
_root_logger = logging.getLogger()
_root_logger.setLevel(logging.INFO)
_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(_fmt)
_fh = logging.handlers.RotatingFileHandler(
    _LOG_FILE, maxBytes=20 * 1024 * 1024, backupCount=5, encoding="utf-8"
)
_fh.setFormatter(_fmt)
_root_logger.addHandler(_sh)
_root_logger.addHandler(_fh)

logger = logging.getLogger("atlas.execution.live_loop")

# ── Constantes ────────────────────────────────────────────────────────────────
# Horario mercado ET: 9:30 - 15:45 (cierre 15 min antes para evitar slippage final)
_MARKET_OPEN_H,  _MARKET_OPEN_M  = 9,  30
_MARKET_CLOSE_H, _MARKET_CLOSE_M = 15, 45
# Cooldown: un simbolo no se repite dentro de este intervalo (minutos)
_SYMBOL_COOLDOWN_MIN = 30


# ── Helpers HTTP ──────────────────────────────────────────────────────────────

def _api_call(
    base: str,
    path: str,
    method: str = "GET",
    body: dict | None = None,
    api_key: str = "atlas-quant-local",
    timeout: int = 25,
) -> dict | None:
    url = f"{base}{path}"
    headers = {"x-api-key": api_key, "Content-Type": "application/json"}
    data = json.dumps(body).encode() if body else None
    try:
        req = Request(url, data=data, headers=headers, method=method)
        with urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (URLError, HTTPError) as e:
        logger.debug("API %s %s: %s", method, path, e)
        return None
    except Exception as e:
        logger.debug("API error %s %s: %s", method, path, e)
        return None


def _equity_strategy_type(direction: str) -> str:
    return "equity_short" if direction in {"bajista", "bearish", "short", "down", "sell"} else "equity_long"


def _load_positions_snapshot(base: str, api_key: str) -> dict:
    payload = _api_call(base, "/api/v2/quant/positions?account_scope=paper", timeout=15, api_key=api_key)
    if payload and payload.get("ok") and isinstance(payload.get("data"), dict):
        return payload["data"]
    payload = _api_call(base, "/positions?account_scope=paper", timeout=15, api_key=api_key)
    if payload and payload.get("ok") and isinstance(payload.get("data"), dict):
        return payload["data"]
    return {}


def _positions_open_symbols(snapshot: dict) -> set[str]:
    symbols: set[str] = set()
    for item in snapshot.get("positions", []):
        if not isinstance(item, dict):
            continue
        for key in ("underlying", "symbol"):
            value = str(item.get(key) or "").strip().upper()
            if value:
                symbols.add(value)
    return symbols


def _reconciliation_state(snapshot: dict) -> str:
    reconciliation = snapshot.get("reconciliation") or {}
    if not isinstance(reconciliation, dict):
        return ""
    return str(reconciliation.get("state") or "").strip().lower()


def _detect_base(ports: list[int], api_key: str) -> str | None:
    for port in ports:
        base = f"http://127.0.0.1:{port}"
        r = _api_call(base, "/health", timeout=5, api_key=api_key)
        if r and r.get("status") == "ok":
            # Preferir el que responde scanner
            scan = _api_call(base, "/scanner/report", timeout=8, api_key=api_key)
            if scan and scan.get("ok"):
                return base
    # Fallback: primer que responde health
    for port in ports:
        base = f"http://127.0.0.1:{port}"
        r = _api_call(base, "/health", timeout=5, api_key=api_key)
        if r and r.get("status") == "ok":
            return base
    return None


def _selector_proposal(base: str, api_key: str, cand: dict) -> dict:
    body = {
        "candidate": {
            "symbol": str(cand.get("symbol") or ""),
            "timeframe": str(cand.get("timeframe") or "1h"),
            "direction": str(cand.get("direction") or "long"),
            "price": cand.get("price"),
            "strategy_key": cand.get("strategy_key"),
            "strategy_label": cand.get("strategy_label"),
            "selection_score": float(cand.get("selection_score") or 0.0),
            "local_win_rate_pct": float(cand.get("local_win_rate_pct") or 0.0),
            "predicted_move_pct": float(cand.get("predicted_move_pct") or 0.0),
            "relative_strength_pct": float(cand.get("relative_strength_pct") or 0.0),
            "order_flow": cand.get("order_flow") or {},
            "confirmation": cand.get("confirmation") or {},
            "why_selected": cand.get("why_selected") or [],
        },
        "account_scope": "paper",
        "chart_provider": "tradingview",
        "prefer_defined_risk": True,
        "allow_equity": True,
        "allow_credit": True,
        "risk_budget_pct": 0.75,
    }
    result = _api_call(base, "/api/v2/quant/selector/proposal", method="POST", body=body, timeout=25, api_key=api_key)
    if not result:
        result = _api_call(base, "/selector/proposal", method="POST", body=body, timeout=25, api_key=api_key)
    return result or {}


# ── Gate de horario de mercado ────────────────────────────────────────────────

def _now_et() -> datetime:
    """Hora actual en ET (UTC-4 EDT / UTC-5 EST). Aproximacion sin pytz."""
    utc = datetime.now(timezone.utc)
    # EDT: segunda domingo de marzo a primera domingo de noviembre
    # Aproximacion simple: UTC-4 de marzo a noviembre, UTC-5 el resto
    month = utc.month
    offset = -4 if 3 <= month <= 11 else -5
    return utc + timedelta(hours=offset)


def _is_market_open(override: bool = False) -> bool:
    if override:
        return True
    now = _now_et()
    # Saltar fines de semana
    if now.weekday() >= 5:
        return False
    market_open  = now.replace(hour=_MARKET_OPEN_H,  minute=_MARKET_OPEN_M,  second=0, microsecond=0)
    market_close = now.replace(hour=_MARKET_CLOSE_H, minute=_MARKET_CLOSE_M, second=0, microsecond=0)
    return market_open <= now <= market_close


def _minutes_to_open() -> float:
    """Minutos hasta la proxima apertura del mercado."""
    now = _now_et()
    if now.weekday() >= 5:
        days_ahead = 7 - now.weekday()
    else:
        days_ahead = 0
    target = now.replace(hour=_MARKET_OPEN_H, minute=_MARKET_OPEN_M, second=0, microsecond=0)
    target += timedelta(days=days_ahead)
    if target <= now:
        target += timedelta(days=1)
        while target.weekday() >= 5:
            target += timedelta(days=1)
    return max(0.0, (target - now).total_seconds() / 60)


# ── Loop principal ────────────────────────────────────────────────────────────

def run_loop(
    base: str,
    interval_sec: int,
    max_per_cycle: int,
    api_key: str,
    no_market_hours_gate: bool = False,
    no_vision_gate: bool = False,
    min_visual_conf: float = 0.0,
    min_signal_score: float = 0.55,
) -> None:
    logger.info("=" * 60)
    logger.info("ATLAS autonomous_loop v4 iniciado")
    logger.info("  base=%s interval=%ds max_per_cycle=%d", base, interval_sec, max_per_cycle)
    logger.info("  market_hours_gate=%s", not no_market_hours_gate)
    logger.info("  symbol_cooldown=%dmin", _SYMBOL_COOLDOWN_MIN)
    logger.info("  signal_score_gate=%.2f (SKIP si score<%.2f)", min_signal_score, min_signal_score)
    logger.info("  vision_gate=%s min_visual_conf=%.2f available=%s",
                not no_vision_gate, min_visual_conf, _VISION_AVAILABLE)
    logger.info("=" * 60)

    cycle = 0
    total_submitted = 0
    # Cooldown: {symbol -> datetime ultima orden}
    symbol_last_order: dict[str, datetime] = {}

    while True:
        now_et = _now_et()

        # ── Gate horario de mercado ───────────────────────────────────────────
        if not _is_market_open(override=no_market_hours_gate):
            mins = _minutes_to_open()
            if cycle == 0 or int(mins) % 30 == 0:  # log cada 30 min
                logger.info(
                    "Mercado cerrado (%s ET). Proximo ciclo en %.0f min.",
                    now_et.strftime("%H:%M"), mins
                )
            sleep_sec = min(300, max(30, (mins - 1) * 60)) if mins > 1 else 60
            time.sleep(sleep_sec)
            continue

        cycle += 1
        t_cycle = time.perf_counter()
        logger.info(
            "[cycle %d] %s ET -- %s UTC",
            cycle,
            now_et.strftime("%H:%M:%S"),
            datetime.now(timezone.utc).strftime("%H:%M:%S"),
        )

        # ── Verificar estado operacional ──────────────────────────────────────
        op = _api_call(base, "/api/v2/quant/operation/status", timeout=10, api_key=api_key)
        if not op or not op.get("ok"):
            op = _api_call(base, "/operation/status", timeout=10, api_key=api_key)

        if op and op.get("data"):
            d = op["data"]
            auton_mode   = d.get("config", {}).get("auton_mode", "unknown")
            auton_active = d.get("auton_mode_active", False)
            failsafe     = d.get("failsafe", {}).get("active", False)

            if failsafe:
                logger.warning("[cycle %d] FAILSAFE activo - saltando", cycle)
                time.sleep(interval_sec)
                continue

            if not auton_active or auton_mode != "paper_autonomous":
                logger.warning("[cycle %d] auton_mode=%s active=%s - no autonomo", cycle, auton_mode, auton_active)
                time.sleep(interval_sec)
                continue

        else:
            logger.warning("[cycle %d] Sin estado operacional - continuando de todas formas", cycle)
        positions_snapshot = _load_positions_snapshot(base, api_key)
        reconciliation_state = _reconciliation_state(positions_snapshot)
        if reconciliation_state != "healthy":
            logger.warning(
                "[cycle %d] Reconciliation en estado '%s' - veto de submit",
                cycle,
                reconciliation_state or "unknown",
            )
            time.sleep(interval_sec)
            continue
        open_symbols = _positions_open_symbols(positions_snapshot)

        # ── Obtener candidatos ────────────────────────────────────────────────
        scan = _api_call(base, "/scanner/report", timeout=15, api_key=api_key)
        if not scan or not scan.get("ok"):
            scan = _api_call(base, "/api/v2/quant/scanner/report", timeout=15, api_key=api_key)

        if not scan or not scan.get("data"):
            logger.info("[cycle %d] Sin candidatos del scanner", cycle)
            time.sleep(interval_sec)
            continue

        candidates = scan["data"].get("candidates", [])
        if not candidates:
            logger.info("[cycle %d] Scanner sin candidatos activos", cycle)
            time.sleep(interval_sec)
            continue

        # Ordenar por score
        sorted_cands = sorted(
            candidates,
            key=lambda c: float(c.get("selection_score") or 0),
            reverse=True,
        )

        # ── Filtrar cooldown ──────────────────────────────────────────────────
        now_utc = datetime.now(timezone.utc)
        cooldown_td = timedelta(minutes=_SYMBOL_COOLDOWN_MIN)
        eligible = []
        for c in sorted_cands:
            sym = str(c.get("symbol") or "").strip()
            if not sym:
                continue
            last = symbol_last_order.get(sym)
            if last and (now_utc - last) < cooldown_td:
                mins_left = int((cooldown_td - (now_utc - last)).total_seconds() / 60)
                logger.debug("  %s en cooldown (%d min restantes)", sym, mins_left)
                continue
            eligible.append(c)
            if len(eligible) >= max_per_cycle:
                break

        if not eligible:
            cooldown_syms = [str(c.get("symbol","")) for c in sorted_cands[:3]]
            logger.info("[cycle %d] Todos los candidatos en cooldown: %s", cycle, ", ".join(cooldown_syms))
            time.sleep(interval_sec)
            continue

        logger.info(
            "[cycle %d] %d candidatos | %d elegibles (cooldown %dmin) -> procesando top %d",
            cycle, len(candidates), len(eligible), _SYMBOL_COOLDOWN_MIN, len(eligible)
        )

        # ── Enviar ordenes ────────────────────────────────────────────────────
        cycle_submitted = 0
        for cand in eligible:
            sym       = str(cand.get("symbol") or "").strip()
            score     = float(cand.get("selection_score") or 0)
            direction = str(cand.get("direction") or "long").lower()
            side      = "buy" if direction in {"long", "alcista", "bull", "up"} else "sell_short"
            tf        = cand.get("timeframe", "?")

            if sym.upper() in open_symbols:
                logger.info("  SKIP    %s | open_symbol_guard ya tiene exposicion abierta", sym)
                continue

            logger.info("[cycle %d] SCANNER %s | scanner_score=%.1f dir=%s tf=%s",
                        cycle, sym, score, direction, tf)

            # ── Gate 1: signal_score compuesto (motif+TIN+MTF+regime) ─────────
            sig_ok, sig_score, sig_log = _signal_gate(cand, min_signal_score)
            logger.info("  SIGNAL  %s | %s", sym, sig_log)
            if not sig_ok:
                logger.info("  SKIP    %s | signal_score %.3f < %.2f (SKIP tier)",
                            sym, sig_score, min_signal_score)
                continue

            # ── Gate 2: validacion visual (color chart Insta360/desktop) ──────
            if not no_vision_gate and _VISION_AVAILABLE:
                vis_ok, vis_reason = _visual_check(
                    direction=direction,
                    api_price=float(cand.get("price") or cand.get("entry_price") or 0),
                    min_conf=min_visual_conf,
                )
                logger.info("  VISION  %s | %s", sym, vis_reason)
                if not vis_ok:
                    logger.info("  SKIP    %s | bloqueado por vision", sym)
                    continue

            proposal = _selector_proposal(base, api_key, cand)
            proposal_data = proposal.get("data") or {}
            order_seed = dict(proposal_data.get("order_seed") or {})
            if not order_seed:
                order_seed = {
                    "symbol": sym,
                    "side": side,
                    "size": 1,
                    "order_type": "market",
                    "account_scope": "paper",
                    "preview": False,
                    "asset_class": "equity",
                    "strategy_type": _equity_strategy_type(direction),
                    "tag": f"autonomous_loop:{cand.get('strategy_key') or 'scanner'}:{cand.get('timeframe') or 'tf'}",
                }
            asset_class = str(order_seed.get("asset_class") or "equity").lower()
            if asset_class == "equity" and not order_seed.get("strategy_type"):
                order_seed["strategy_type"] = _equity_strategy_type(direction)
            order_seed["preview"] = False

            order_body = {
                "order": order_seed,
                "action": "submit",
                "capture_context": bool(order_seed.get("chart_plan") or order_seed.get("camera_plan")) and not no_vision_gate,
            }

            result = _api_call(
                base, "/api/v2/quant/operation/test-cycle",
                method="POST", body=order_body, timeout=25, api_key=api_key,
            )
            if not result:
                result = _api_call(
                    base, "/operation/test-cycle",
                    method="POST", body=order_body, timeout=25, api_key=api_key,
                )

            if result and result.get("ok"):
                data_r   = result.get("data", {})
                blocked  = data_r.get("blocked", False)
                decision = data_r.get("decision", "?")
                reasons  = data_r.get("reasons", [])
                exec_r   = data_r.get("execution", {}) or {}
                tradier  = (exec_r.get("response") or {}).get("tradier_response") or {}
                order_id = tradier.get("id")

                if blocked:
                    reason_str = ", ".join(str(r) for r in reasons[:2]) if reasons else "?"
                    logger.info("  BLOCKED %s | %s | %s", sym, decision, reason_str)
                elif order_id or (not blocked and decision not in ("", None)):
                    cycle_submitted += 1
                    total_submitted += 1
                    symbol_last_order[sym] = now_utc
                    logger.info(
                        "  SUBMIT  %s | id=%s | decision=%s | score=%.1f",
                        sym, order_id or "local", decision, score
                    )

                    # ── Registrar en Paper Broker local ───────────────────────
                    _paper_record_fill(
                        base=base, api_key=api_key,
                        symbol=sym, side=side,
                        qty=int(order_body["order"].get("size", 1)),
                        price=float(cand.get("price") or cand.get("entry_price") or 0),
                        strategy=str(cand.get("setup_type") or cand.get("strategy") or "auto"),
                        signal_score=sig_score,
                        score_tier=str(cand.get("score_tier") or
                                       ("FULL" if sig_score >= 0.75 else
                                        "NORMAL" if sig_score >= 0.65 else
                                        "SMALL" if sig_score >= 0.55 else "SKIP")),
                        option_strategy=str(cand.get("option_strategy_type") or ""),
                    )
            else:
                logger.warning("  ERROR   %s | sin respuesta del servidor", sym)

        elapsed_ms = (time.perf_counter() - t_cycle) * 1000
        logger.info(
            "[cycle %d] FIN | submitted=%d | total=%d | %.0fms",
            cycle, cycle_submitted, total_submitted, elapsed_ms
        )
        time.sleep(interval_sec)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS autonomous loop v3")
    parser.add_argument("--port",               type=int,   default=None)
    parser.add_argument("--interval",           type=int,   default=120,  help="Segundos entre ciclos (default 120)")
    parser.add_argument("--max-per-cycle",      type=int,   default=1)
    parser.add_argument("--api-key",            default="atlas-quant-local")
    parser.add_argument("--no-market-hours-gate", action="store_true", help="Operar fuera de horario (testing)")
    parser.add_argument("--no-vision-gate",     action="store_true", help="Deshabilitar validacion visual")
    parser.add_argument("--min-visual-conf",    type=float, default=0.0,
                        help="Confianza minima visual para bloquear (0.0=solo log, 0.6=bloqueo)")
    parser.add_argument("--min-signal-score",   type=float, default=0.55,
                        help="Signal score minimo para enviar orden (0.55=SMALL+, 0.0=desactivado)")
    args = parser.parse_args()

    # Inicializar pipeline visual (no bloquea si falla)
    _init_vision()

    if args.port:
        base = f"http://127.0.0.1:{args.port}"
        r = _api_call(base, "/health", timeout=5, api_key=args.api_key)
        if not r or r.get("status") != "ok":
            print(f"ERROR: servidor no responde en puerto {args.port}")
            return 1
    else:
        base = _detect_base([8792, 8795], args.api_key)
        if not base:
            print("ERROR: no se encontro servidor Quant activo en puertos 8792/8795")
            return 1

    logger.info("Servidor: %s", base)

    try:
        run_loop(
            base, args.interval, args.max_per_cycle, args.api_key,
            args.no_market_hours_gate,
            no_vision_gate=args.no_vision_gate,
            min_visual_conf=args.min_visual_conf,
            min_signal_score=args.min_signal_score,
        )
    except KeyboardInterrupt:
        logger.info("Loop detenido (Ctrl+C)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
