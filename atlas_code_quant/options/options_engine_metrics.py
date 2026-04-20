"""Métricas Prometheus y snapshot UI para Options Engine (paper-only).

Actualizado desde orquestador, journal, AutoCloseEngine y sincronización periódica
(vía ``GrafanaDashboard.sync_from_canonical``).
"""
from __future__ import annotations

import json
import logging
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger("atlas.options.engine_metrics")

try:
    from prometheus_client import Counter, Gauge

    _PROM_OK = True
except ImportError:  # pragma: no cover
    Counter = Gauge = None  # type: ignore[misc, assignment]
    _PROM_OK = False

_PIPELINE_MODULES = ("briefing", "intent_router", "entry_planner", "journal", "autoclose")

_state: dict[str, Any] = {
    "last_session_plan": None,
    "last_journal_path": None,
    "last_iv_rank_quality_score": None,
    "last_options_flow": None,
    "last_visual_signal": None,
    "last_options_self_audit": None,
    "last_paper_performance": None,
    "utc_day": "",
    "journal_events_today": 0,
    "journal_sessions_today": 0,
    "last_journal_write_ts": 0.0,
    "module_last_ts": {m: 0.0 for m in _PIPELINE_MODULES},
    "module_status": {m: 0.0 for m in _PIPELINE_MODULES},
    "last_sentinel_snapshot": None,
}

_m: dict[str, Any] = {}


def compute_iv_rank_quality_score(briefing: dict[str, Any] | None) -> float:
    """Deriva un score 0..1 **conservador** desde el briefing (IVRankCalculator → session_briefing).

    Fuentes: ``iv_rank_payload_quality``, ``iv_source.quality``, ``iv_source.error``,
    ``quality_flags`` del briefing (p.ej. ``iv_rank_quality_approx``, ``iv_rank_fallback_mid``).

    Mapping (conservador: sin dato nunca se asume “ok” neto):
    - ``iv_source.error`` presente → **0.0** (fallo explícito).
    - Sin cadena ``quality`` efectiva (vacía) → **0.25** (desconocido / insuficiente dato).
    - ``quality == "ok"`` y **ningún** flag IV-degradado en ``quality_flags`` → **1.0**.
    - ``quality == "ok"`` pero hay flags IV-degradados → **0.5**.
    - ``approx`` o ``insufficient_history`` → **0.5** (peor que ok neto).
    - Cualquier otro valor de ``quality`` → **0.25**.

    Flags IV-degradados reconocidos: prefijo ``iv_rank_quality_``, ``approx_iv_rank``,
    ``iv_rank_fallback_mid``, ``iv_hv_ratio_fallback_unit``, ``iv_current_missing``, ``spot_unavailable``.
    """
    if not briefing:
        return 0.25
    iv_src = briefing.get("iv_source")
    if isinstance(iv_src, dict) and iv_src.get("error"):
        return 0.0
    q_top = str(briefing.get("iv_rank_payload_quality") or "").strip().lower()
    q_inner = ""
    if isinstance(iv_src, dict):
        q_inner = str(iv_src.get("quality") or "").strip().lower()
    raw = q_top or q_inner
    flags = list(briefing.get("quality_flags") or [])

    def _iv_degrade_flag(f: str) -> bool:
        fs = str(f)
        if fs.startswith("iv_rank_quality_"):
            return True
        return fs in (
            "approx_iv_rank",
            "iv_rank_fallback_mid",
            "iv_hv_ratio_fallback_unit",
            "iv_current_missing",
            "spot_unavailable",
        )

    degraded = any(_iv_degrade_flag(x) for x in flags)
    if not raw:
        return 0.25
    if raw == "ok":
        return 0.5 if degraded else 1.0
    if raw in ("approx", "insufficient_history"):
        return 0.5
    return 0.25


def get_last_iv_rank_quality_score() -> float | None:
    """Último score calculado (útil en tests y depuración)."""
    v = _state.get("last_iv_rank_quality_score")
    return float(v) if isinstance(v, (int, float)) else None


def _utc_day() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%d")


def _ensure_day() -> None:
    d = _utc_day()
    if _state["utc_day"] != d:
        _state["utc_day"] = d
        _state["journal_events_today"] = 0
        _state["journal_sessions_today"] = 0


_FLOW_SENTINEL = -1.0
"""Valor gauge cuando no hay dato interpretable (no confundir con señal neutral del provider)."""


def _finite_float(x: Any) -> float | None:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    if math.isnan(v) or math.isinf(v):
        return None
    return v


def options_flow_bridge_enabled() -> bool:
    """Si es verdadero, ``PaperSessionOrchestrator`` intenta ``OptionsFlowProvider.build_snapshot`` (Tradier)."""
    return str(os.environ.get("ATLAS_OPTIONS_FLOW_BRIDGE", "")).strip().lower() in ("1", "true", "yes", "on")


def _options_flow_snapshot_coherent(snapshot: dict[str, Any]) -> bool:
    """Payload mínimo para publicar ratios sin inventar: ``available`` y campos numéricos clave del snapshot."""
    if not isinstance(snapshot, dict) or not bool(snapshot.get("available")):
        return False
    for key in ("put_call_volume_ratio", "gamma_bias_pct", "score_pct", "confidence_pct"):
        if _finite_float(snapshot.get(key)) is None:
            return False
    return True


def try_fetch_options_flow_snapshot(symbol: str, *, price_hint: float = 0.0) -> dict[str, Any]:
    """Construye snapshot vía ``OptionsFlowProvider`` (mismo contrato que el scanner). Errores → dict ``available: False``."""
    sym = str(symbol or "").strip().upper()
    if not sym:
        return {"available": False, "reason": "empty_symbol", "scope": "paper", "mode": "options_flow"}
    try:
        from scanner.options_flow_provider import OptionsFlowProvider
    except ModuleNotFoundError:  # pragma: no cover
        from atlas_code_quant.scanner.options_flow_provider import OptionsFlowProvider

    try:
        from backtesting.winning_probability import TradierClient
    except ModuleNotFoundError:  # pragma: no cover
        from atlas_code_quant.backtesting.winning_probability import TradierClient

    client = TradierClient(scope="paper")
    prov = OptionsFlowProvider()
    return prov.build_snapshot(symbol=sym, client=client, scope="paper", price_hint=float(price_hint or 0.0))


def apply_options_flow_snapshot_to_metrics(
    symbol: str,
    snapshot: dict[str, Any] | None,
) -> dict[str, Any]:
    """Publica gauges ``atlas_options_flow_*`` desde un snapshot del provider (o estado conservador si falta/incompleto).

    Valores neutrales / sin señal:
    - ``atlas_options_flow_payload_available{symbol}=0`` si no hay snapshot coherente.
    - Ratios y score usan **-1** como centinela (no usar 1.0/50.0 del provider fuera de contexto).
    - ``confidence_pct`` = 0 cuando no hay payload coherente.
    """
    _init_prom()
    _ensure_flow_gauges()
    sym = str(symbol or "").strip().upper() or "UNKNOWN"
    coherent = _options_flow_snapshot_coherent(snapshot) if snapshot is not None else False

    if not coherent:
        row = {
            "symbol": sym,
            "payload_available": 0.0,
            "put_call_volume_ratio": _FLOW_SENTINEL,
            "gamma_bias_pct": _FLOW_SENTINEL,
            "score_pct": _FLOW_SENTINEL,
            "confidence_pct": 0.0,
            "coherent": False,
        }
        _state["last_options_flow"] = row
        if _m:
            try:
                _m["flow_payload_available"].labels(symbol=sym).set(0.0)
                _m["flow_put_call_volume_ratio"].labels(symbol=sym).set(_FLOW_SENTINEL)
                _m["flow_gamma_bias_pct"].labels(symbol=sym).set(_FLOW_SENTINEL)
                _m["flow_score_pct"].labels(symbol=sym).set(_FLOW_SENTINEL)
                _m["flow_confidence_pct"].labels(symbol=sym).set(0.0)
            except Exception as exc:
                logger.debug("apply_options_flow_snapshot_to_metrics (degraded): %s", exc)
        update_options_engine_sentinels()
        return row

    pcr = _finite_float(snapshot.get("put_call_volume_ratio"))
    gamma = _finite_float(snapshot.get("gamma_bias_pct"))
    score = _finite_float(snapshot.get("score_pct"))
    conf = _finite_float(snapshot.get("confidence_pct"))
    if pcr is None or gamma is None or score is None or conf is None:
        return apply_options_flow_snapshot_to_metrics(symbol, None)

    row = {
        "symbol": sym,
        "payload_available": 1.0,
        "put_call_volume_ratio": pcr,
        "gamma_bias_pct": gamma,
        "score_pct": score,
        "confidence_pct": conf,
        "coherent": True,
    }
    _state["last_options_flow"] = row
    if _m:
        try:
            _m["flow_payload_available"].labels(symbol=sym).set(1.0)
            _m["flow_put_call_volume_ratio"].labels(symbol=sym).set(pcr)
            _m["flow_gamma_bias_pct"].labels(symbol=sym).set(gamma)
            _m["flow_score_pct"].labels(symbol=sym).set(score)
            _m["flow_confidence_pct"].labels(symbol=sym).set(conf)
        except Exception as exc:
            logger.debug("apply_options_flow_snapshot_to_metrics: %s", exc)
    update_options_engine_sentinels()
    return row


def get_last_options_flow_metrics() -> dict[str, Any] | None:
    v = _state.get("last_options_flow")
    return dict(v) if isinstance(v, dict) else None


def compute_visual_signal_state_score(visual: dict[str, Any] | None) -> float:
    """Estado observable VisualSignalAdapter → un solo gauge 0..1 (más alto = más materialidad de riesgo).

    - **0.0** error del adaptador o payload inválido.
    - **0.2** sin spot / input insuficiente (no se interpreta contexto útil).
    - **0.5** análisis ok sin ruptura detectada.
    - **0.75** ruptura no crítica.
    - **1.0** ruptura crítica (p. ej. short strike en 0DTE).

    Ausencia o fallo **no** se mapea a valores altos.
    """
    if not visual or not isinstance(visual, dict):
        return 0.2
    av = str(visual.get("availability") or "").strip().lower()
    if av == "error":
        return 0.0
    if av == "unavailable":
        return 0.2
    if av != "ok":
        return 0.2
    if bool(visual.get("critical_breach")):
        return 1.0
    if bool(visual.get("breach_detected")):
        return 0.75
    return 0.5


def get_last_visual_signal() -> dict[str, Any] | None:
    v = _state.get("last_visual_signal")
    return dict(v) if isinstance(v, dict) else None


def compute_options_self_audit_state_score(block: dict[str, Any] | None) -> float:
    """Único gauge ``atlas_options_self_audit_state`` (0..1), conservador.

    - **0.0** ``status=error`` (excepción o audit no ejecutable con evidencia de fallo).
    - **0.1** ausencia de bloque ``options_self_audit`` en el plan (no se asume éxito).
    - **0.25** ``status=skipped`` (deshabilitado por alcance/env; no es corrida completa).
    - **0.35** ``status=ok`` y ``passed`` falso (rollup ERROR/BLOCK).
    - **0.72** ``status=ok``, ``passed`` verdadero y ``overall_severity=WARN`` (o findings sin bloqueo).
    - **1.0** ``status=ok``, ``passed`` verdadero y severidad máxima ``INFO`` (sin hallazgos graves).

    No se mapea a 1.0 si el audit no corrió o está omitido.
    """
    if not block or not isinstance(block, dict):
        return 0.1
    st = str(block.get("status") or "").strip().lower()
    if st == "error":
        return 0.0
    if st == "skipped":
        return 0.25
    if st != "ok":
        return 0.1
    passed = block.get("passed")
    if passed is False:
        return 0.35
    if passed is not True:
        return 0.1
    sev = str(block.get("overall_severity") or "INFO").strip().upper()
    if sev == "WARN":
        return 0.72
    if sev == "INFO":
        return 1.0
    return 0.5


def get_last_options_self_audit() -> dict[str, Any] | None:
    v = _state.get("last_options_self_audit")
    return dict(v) if isinstance(v, dict) else None


def extract_close_realized_pnls_from_journal_text(text: str) -> list[float]:
    """Orden de archivo: un float por cada ``close_execution`` con ``pnl_realized`` numérico finito."""
    out: list[float] = []
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if str(row.get("event_type") or "") != "close_execution":
            continue
        payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
        val = payload.get("pnl_realized")
        if val is None:
            continue
        try:
            v = float(val)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(v):
            continue
        out.append(v)
    return out


def compute_paper_performance_from_pnls(pnls: list[float]) -> dict[str, Any]:
    """Métricas conservadoras desde PnL realizados por cierre (journal ``close_execution``).

    - **Win rate:** ``wins / n`` con ``n = len(pnls)``; si ``n < 2`` → ``win_rate_ratio = -1`` (muestra insuficiente).
    - **Profit factor:** ``sum(wins gross) / abs(sum(losses gross))``; si no hay pérdidas y sí ganancias → **99.0**
      (tope documentado, no infinito); si no hay ganancias ni pérdidas útiles → ``-1``; solo pérdidas → ``0.0``.
    - **Net PnL:** suma de todos los elementos (puede ser 0).
    - **Max drawdown (USD):** sobre equity acumulada empezando en 0; ``max_t(peak_t - equity_t)``, valor **≥ 0**.
    """
    n = len(pnls)
    if n == 0:
        return {
            "n_closes": 0,
            "win_rate_ratio": -1.0,
            "profit_factor": -1.0,
            "net_realized_pnl_usd": 0.0,
            "max_drawdown_usd": 0.0,
            "insufficient_win_rate_sample": True,
            "insufficient_profit_factor": True,
            "insufficient_profit_factor_capped_no_losses": False,
        }
    wins_n = sum(1 for p in pnls if p > 0)
    gross_win = sum(p for p in pnls if p > 0)
    gross_loss = sum(p for p in pnls if p < 0)
    loss_abs = abs(gross_loss)
    net = float(sum(pnls))
    cum = 0.0
    peak = 0.0
    max_dd = 0.0
    for p in pnls:
        cum += p
        peak = max(peak, cum)
        max_dd = max(max_dd, peak - cum)
    win_rate_ratio = -1.0 if n < 2 else float(wins_n) / float(n)
    if loss_abs > 1e-12:
        profit_factor = min(gross_win / loss_abs, 99.0)
        insufficient_pf = False
    elif gross_win > 1e-12:
        profit_factor = 99.0
        insufficient_pf = True
    elif gross_loss < -1e-12:
        profit_factor = 0.0
        insufficient_pf = False
    else:
        profit_factor = -1.0
        insufficient_pf = True
    return {
        "n_closes": n,
        "win_rate_ratio": win_rate_ratio,
        "profit_factor": profit_factor,
        "net_realized_pnl_usd": round(net, 4),
        "max_drawdown_usd": round(max_dd, 4),
        "insufficient_win_rate_sample": n < 2,
        "insufficient_profit_factor": profit_factor < 0,
        "insufficient_profit_factor_capped_no_losses": profit_factor == 99.0,
    }


def get_last_paper_performance() -> dict[str, Any] | None:
    v = _state.get("last_paper_performance")
    return dict(v) if isinstance(v, dict) else None


def _module_age_seconds(module: str, *, now: float | None = None) -> float:
    """Edad en segundos desde ``module_last_ts``; sin marca → ~infinito."""
    t = time.time() if now is None else float(now)
    ts = float(_state["module_last_ts"].get(module) or 0.0)
    if ts <= 0.0:
        return 1e9
    return max(0.0, t - ts)


def _score_metrics_freshness(*, now: float | None = None) -> float:
    """Sentinela pipeline núcleo (briefing, intent, entry, journal): max edad conservadora."""
    mods = ("briefing", "intent_router", "entry_planner", "journal")
    ages = [_module_age_seconds(m, now=now) for m in mods]
    mx = max(ages) if ages else 1e9
    if mx >= 1e8:
        return 0.25
    if mx >= 86400:
        return 0.0
    if mx >= 3600:
        return 0.25
    if mx >= 900:
        return 0.5
    if mx >= 120:
        return 0.75
    return 1.0


def _score_journal_heartbeat(*, now: float | None = None) -> float:
    """Sentinela escritura journal: edad desde último write conocido (mtime o append)."""
    t = time.time() if now is None else float(now)
    lw = float(_state.get("last_journal_write_ts") or 0.0)
    if lw <= 0.0:
        return 0.25
    age = max(0.0, t - lw)
    if age >= 86400:
        return 0.0
    if age >= 3600:
        return 0.33
    if age >= 300:
        return 0.66
    return 1.0


def _score_autoclose_activity(*, now: float | None = None) -> float:
    """Sentinela AutoClose: best-effort — silencio sospechoso si hay trades abiertos.

    Limitación: no se conoce el intervalo esperado de evaluación; umbrales conservadores.
    Sin posiciones abiertas, silencio prolongado no penaliza fuerte.
    """
    ac_age = _module_age_seconds("autoclose", now=now)
    open_n = int(_state.get("paper_open_trades") or 0)
    if open_n <= 0:
        if ac_age >= 1e8 - 1:
            return 0.5
        if ac_age <= 604800:
            return 1.0
        return 0.75
    if ac_age >= 1e8 - 1:
        return 0.25
    if ac_age <= 600:
        return 1.0
    if ac_age <= 3600:
        return 0.66
    if ac_age <= 86400:
        return 0.33
    return 0.0


def _score_options_flow_sentinel() -> float:
    """Sentinela options flow / scanner bridge: coherencia del último snapshot."""
    if not options_flow_bridge_enabled():
        return 0.25
    row = _state.get("last_options_flow")
    if not isinstance(row, dict):
        return 0.25
    if bool(row.get("coherent")):
        return 1.0
    return 0.0


def _score_iv_rank_sentinel() -> float:
    """Sentinela degradación IV: deriva del último ``iv_rank_quality_score`` calculado."""
    v = _state.get("last_iv_rank_quality_score")
    if v is None:
        return 0.25
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.25
    if f >= 0.9:
        return 1.0
    if f >= 0.5:
        return 0.66
    if f >= 0.25:
        return 0.33
    return 0.0


def _ensure_sentinel_gauges() -> None:
    """Gauges explícitos 0..1 (1=saludable, 0=crítico, 0.25≈unknown/N/A)."""
    global _m
    if not _PROM_OK or not _m:
        return
    specs = (
        ("sentinel_metrics_freshness", "atlas_options_sentinel_metrics_freshness", "Freshness módulos núcleo pipeline"),
        ("sentinel_journal_heartbeat", "atlas_options_sentinel_journal_heartbeat", "Heartbeat journal (edad escritura)"),
        ("sentinel_autoclose_activity", "atlas_options_sentinel_autoclose_activity", "AutoClose vs silencio (best-effort)"),
        ("sentinel_options_flow", "atlas_options_sentinel_options_flow", "Options flow bridge / snapshot coherente"),
        ("sentinel_iv_rank_quality", "atlas_options_sentinel_iv_rank_quality", "IV rank quality degradación"),
    )
    for key, prom_name, desc in specs:
        if key in _m:
            continue
        try:
            _m[key] = Gauge(prom_name, desc)
        except Exception as exc:
            logger.debug("_ensure_sentinel_gauges %s: %s", prom_name, exc)


def update_options_engine_sentinels(*, now: float | None = None) -> dict[str, float]:
    """Recalcula sentinelas explícitos y publica gauges + ``_state['last_sentinel_snapshot']``.

    Escala **conservadora** por sentinela (0..1): valores altos = mejor; **0.25** suele indicar
    dato ausente o función desactivada (no “OK” implícito).
    """
    _init_prom()
    _ensure_sentinel_gauges()
    scores = {
        "metrics_freshness": float(_score_metrics_freshness(now=now)),
        "journal_heartbeat": float(_score_journal_heartbeat(now=now)),
        "autoclose_activity": float(_score_autoclose_activity(now=now)),
        "options_flow": float(_score_options_flow_sentinel()),
        "iv_rank_quality": float(_score_iv_rank_sentinel()),
    }
    _state["last_sentinel_snapshot"] = dict(scores)
    if not _m:
        return scores
    try:
        _m["sentinel_metrics_freshness"].set(scores["metrics_freshness"])
        _m["sentinel_journal_heartbeat"].set(scores["journal_heartbeat"])
        _m["sentinel_autoclose_activity"].set(scores["autoclose_activity"])
        _m["sentinel_options_flow"].set(scores["options_flow"])
        _m["sentinel_iv_rank_quality"].set(scores["iv_rank_quality"])
    except Exception as exc:
        logger.debug("update_options_engine_sentinels: %s", exc)
    return scores


def get_last_sentinel_snapshot() -> dict[str, float] | None:
    """Último snapshot de sentinelas (tests / auditoría)."""
    v = _state.get("last_sentinel_snapshot")
    return dict(v) if isinstance(v, dict) else None


def _ensure_iv_rank_quality_gauge() -> None:
    """Registra el gauge IV si el proceso arrancó con un ``_m`` antiguo sin esta clave."""
    global _m
    if not _PROM_OK or not _m or "iv_rank_quality_score" in _m:
        return
    try:
        _m["iv_rank_quality_score"] = Gauge(
            "atlas_options_iv_rank_quality_score",
            "Calidad IV rank briefing: 1 ok 0.5 degradado 0.25 desconocido 0 error iv_source",
        )
    except Exception as exc:
        logger.debug("_ensure_iv_rank_quality_gauge: %s", exc)


def _init_prom() -> None:
    global _m
    if not _PROM_OK or _m:
        return
    try:
        _m["go_nogo"] = Gauge(
            "atlas_options_session_go_nogo",
            "Sesión paper: 0=no-go 0.5=degradado 1=go",
        )
        _m["go_nogo_count"] = Counter(
            "atlas_options_session_go_nogo_count",
            "Conteo por decisión de sesión paper",
            ["decision"],
        )
        _m["pipe_status"] = Gauge(
            "atlas_options_pipeline_module_status",
            "Estado módulo pipeline 0=off 0.5=degradado 1=ok",
            ["module"],
        )
        _m["pipe_last_run"] = Gauge(
            "atlas_options_pipeline_module_last_run_seconds",
            "Segundos desde última ejecución del módulo (edad)",
            ["module"],
        )
        _m["journal_events"] = Gauge(
            "atlas_options_journal_events_today",
            "Eventos journal options hoy (UTC)",
        )
        _m["journal_sessions"] = Gauge(
            "atlas_options_journal_sessions_today",
            "Planes de sesión registrados hoy (UTC)",
        )
        _m["journal_write_age"] = Gauge(
            "atlas_options_journal_last_write_age_seconds",
            "Segundos desde última escritura journal",
        )
        _m["journal_size"] = Gauge(
            "atlas_options_journal_file_size_bytes",
            "Tamaño archivo JSONL journal",
        )
        _m["paper_open"] = Gauge(
            "atlas_options_paper_trades_open",
            "Trades paper con entrada y sin cierre (journal)",
        )
        _m["paper_closed"] = Gauge(
            "atlas_options_paper_trades_closed_today",
            "Cierres paper registrados hoy (UTC)",
        )
        _m["paper_phantom"] = Gauge(
            "atlas_options_paper_trades_phantom_today",
            "Phantoms paper hoy (placeholder hasta regla formal)",
        )
        _m["paper_debit_no_stop"] = Gauge(
            "atlas_options_paper_debit_positions_no_stop",
            "Posiciones débito sin stop (placeholder)",
        )
        _m["autoclose_triggers"] = Counter(
            "atlas_options_autoclose_triggers_total",
            "Disparos AutoCloseEngine por razón",
            ["reason"],
        )
        _m["errors"] = Counter(
            "atlas_options_errors_total",
            "Errores options observabilidad",
            ["type"],
        )
        _m["iv_rank_quality_score"] = Gauge(
            "atlas_options_iv_rank_quality_score",
            "Calidad IV rank briefing: 1 ok 0.5 degradado 0.25 desconocido 0 error iv_source",
        )
        _m["self_audit_state"] = Gauge(
            "atlas_options_self_audit_state",
            "Self-audit operativo paper: 0 error 0.1 sin bloque 0.25 skipped 0.35 failed 0.72 WARN 1.0 INFO limpio",
        )
        _m["sentinel_metrics_freshness"] = Gauge(
            "atlas_options_sentinel_metrics_freshness",
            "Sentinela: freshness módulos núcleo pipeline (1=sano … 0=crítico)",
        )
        _m["sentinel_journal_heartbeat"] = Gauge(
            "atlas_options_sentinel_journal_heartbeat",
            "Sentinela: heartbeat journal por edad de escritura",
        )
        _m["sentinel_autoclose_activity"] = Gauge(
            "atlas_options_sentinel_autoclose_activity",
            "Sentinela: AutoClose vs silencio (best-effort)",
        )
        _m["sentinel_options_flow"] = Gauge(
            "atlas_options_sentinel_options_flow",
            "Sentinela: options flow / snapshot coherente",
        )
        _m["sentinel_iv_rank_quality"] = Gauge(
            "atlas_options_sentinel_iv_rank_quality",
            "Sentinela: degradación IV rank quality",
        )
        # Preinicializa series con labels para que los dashboards de Options
        # muestren estado base en arranque en frio y no "No data".
        _m["go_nogo"].set(0.0)
        for decision in ("go", "no_go", "force_no_trade"):
            _m["go_nogo_count"].labels(decision=decision).inc(0)
        for module in _PIPELINE_MODULES:
            _m["pipe_status"].labels(module=module).set(0.0)
            _m["pipe_last_run"].labels(module=module).set(1e9)
        _m["journal_events"].set(0.0)
        _m["journal_sessions"].set(0.0)
        _m["journal_write_age"].set(1e9)
        _m["journal_size"].set(0.0)
        _m["paper_open"].set(0.0)
        _m["paper_closed"].set(0.0)
        _m["paper_phantom"].set(0.0)
        _m["paper_debit_no_stop"].set(0.0)
        _m["iv_rank_quality_score"].set(0.25)
        _m["self_audit_state"].set(0.25)
        _m["sentinel_metrics_freshness"].set(0.25)
        _m["sentinel_journal_heartbeat"].set(0.25)
        _m["sentinel_autoclose_activity"].set(0.5)
        _m["sentinel_options_flow"].set(0.25)
        _m["sentinel_iv_rank_quality"].set(0.25)
    except Exception as exc:  # pragma: no cover
        logger.warning("options_engine_metrics init: %s", exc)
        _m.clear()


def _ensure_flow_gauges() -> None:
    """Registra gauges de flow si el proceso arrancó con un ``_m`` sin estas claves."""
    global _m
    if not _PROM_OK or not _m or "flow_payload_available" in _m:
        return
    try:
        _m["flow_payload_available"] = Gauge(
            "atlas_options_flow_payload_available",
            "1 si snapshot OptionsFlowProvider es coherente (available+numéricos); 0 si ausente/incompleto",
            ["symbol"],
        )
        _m["flow_put_call_volume_ratio"] = Gauge(
            "atlas_options_flow_put_call_volume_ratio",
            "Put/call volumen (front). -1 = sin dato interpretable (ignorar como señal)",
            ["symbol"],
        )
        _m["flow_gamma_bias_pct"] = Gauge(
            "atlas_options_flow_gamma_bias_pct",
            "Sesgo gamma % (front). -1 = sin dato interpretable",
            ["symbol"],
        )
        _m["flow_score_pct"] = Gauge(
            "atlas_options_flow_score_pct",
            "Score interno flow 0..100. -1 = sin dato interpretable",
            ["symbol"],
        )
        _m["flow_confidence_pct"] = Gauge(
            "atlas_options_flow_confidence_pct",
            "Confianza % del snapshot flow; 0 si payload no coherente",
            ["symbol"],
        )
    except Exception as exc:
        logger.debug("_ensure_flow_gauges: %s", exc)


def _ensure_visual_signal_gauge() -> None:
    global _m
    if not _PROM_OK or not _m or "visual_signal_state" in _m:
        return
    try:
        _m["visual_signal_state"] = Gauge(
            "atlas_options_visual_signal_state",
            "VisualSignalAdapter: 0 error 0.2 sin input útil 0.5 ok sin breach 0.75 breach 1.0 crítico",
        )
    except Exception as exc:
        logger.debug("_ensure_visual_signal_gauge: %s", exc)


def _ensure_self_audit_gauge() -> None:
    global _m
    if not _PROM_OK or not _m or "self_audit_state" in _m:
        return
    try:
        _m["self_audit_state"] = Gauge(
            "atlas_options_self_audit_state",
            "Self-audit operativo paper: 0 error 0.1 sin bloque 0.25 skipped 0.35 failed 0.72 WARN 1.0 INFO limpio",
        )
    except Exception as exc:
        logger.debug("_ensure_self_audit_gauge: %s", exc)


def _ensure_paper_performance_gauges() -> None:
    global _m
    if not _PROM_OK or not _m or "paper_win_rate_ratio" in _m:
        return
    try:
        _m["paper_win_rate_ratio"] = Gauge(
            "atlas_options_paper_win_rate_ratio",
            "Win rate cierres paper (journal): 0..1; -1 muestra insuficiente (<2 cierres con PnL)",
        )
        _m["paper_profit_factor"] = Gauge(
            "atlas_options_paper_profit_factor",
            "PF bruto wins/abs(losses); -1 no calculable; 99 tope si no hay pérdidas en muestra",
        )
        _m["paper_net_realized_pnl_usd"] = Gauge(
            "atlas_options_paper_net_realized_pnl_usd",
            "Suma pnl_realized en close_execution (journal)",
        )
        _m["paper_max_drawdown_usd"] = Gauge(
            "atlas_options_paper_max_drawdown_usd",
            "Peor caída desde equity peak acumulada (USD, ≥0)",
        )
        _m["paper_perf_close_count"] = Gauge(
            "atlas_options_paper_performance_close_count",
            "Número de close_execution con pnl_realized numérico usado en performance",
        )
    except Exception as exc:
        logger.debug("_ensure_paper_performance_gauges: %s", exc)


def _apply_paper_performance_gauges(metrics: dict[str, Any]) -> None:
    _init_prom()
    _ensure_paper_performance_gauges()
    _state["last_paper_performance"] = dict(metrics)
    if not _m:
        return
    try:
        _m["paper_win_rate_ratio"].set(float(metrics["win_rate_ratio"]))
        _m["paper_profit_factor"].set(float(metrics["profit_factor"]))
        _m["paper_net_realized_pnl_usd"].set(float(metrics["net_realized_pnl_usd"]))
        _m["paper_max_drawdown_usd"].set(float(metrics["max_drawdown_usd"]))
        _m["paper_perf_close_count"].set(float(metrics["n_closes"]))
    except Exception as exc:
        logger.debug("_apply_paper_performance_gauges: %s", exc)


def record_pipeline_module(*, module: str, status: float) -> None:
    """status: 0.0, 0.5 o 1.0."""
    if module not in _PIPELINE_MODULES:
        return
    _init_prom()
    now = time.time()
    _state["module_last_ts"][module] = now
    _state["module_status"][module] = float(status)
    if not _m:
        return
    try:
        _m["pipe_status"].labels(module=module).set(float(status))
        _m["pipe_last_run"].labels(module=module).set(0.0)
    except Exception as exc:
        logger.debug("record_pipeline_module: %s", exc)


def tick_pipeline_ages() -> None:
    """Actualiza edad desde última corrida de cada módulo (llamar desde sync métricas)."""
    _init_prom()
    now = time.time()
    if _m:
        try:
            for mod in _PIPELINE_MODULES:
                ts = float(_state["module_last_ts"].get(mod) or 0.0)
                age = max(0.0, now - ts) if ts > 0 else 1e9
                _m["pipe_last_run"].labels(module=mod).set(age)
        except Exception as exc:
            logger.debug("tick_pipeline_ages: %s", exc)
    update_options_engine_sentinels(now=now)


def record_session_plan(plan: dict[str, Any], *, journal_path: str | Path | None = None) -> None:
    """Llamar al finalizar ``build_session_plan`` (tras journal si aplica)."""
    _init_prom()
    _ensure_iv_rank_quality_gauge()
    _ensure_visual_signal_gauge()
    _ensure_self_audit_gauge()
    _ensure_day()
    _state["last_session_plan"] = plan
    if journal_path is not None:
        _state["last_journal_path"] = str(journal_path)

    briefing_for_iv = plan.get("briefing") if isinstance(plan.get("briefing"), dict) else {}
    iv_score = compute_iv_rank_quality_score(briefing_for_iv)
    _state["last_iv_rank_quality_score"] = iv_score
    if _m:
        try:
            _m["iv_rank_quality_score"].set(float(iv_score))
        except Exception as exc:
            logger.debug("iv_rank_quality_score: %s", exc)

    sym_flow = str(plan.get("symbol") or "").strip().upper() or "UNKNOWN"
    snap_flow = plan.get("options_flow_snapshot")
    if not isinstance(snap_flow, dict):
        snap_flow = None
    apply_options_flow_snapshot_to_metrics(sym_flow, snap_flow)

    vs_block = briefing_for_iv.get("visual_signal") if isinstance(briefing_for_iv, dict) else None
    vs_score = compute_visual_signal_state_score(vs_block if isinstance(vs_block, dict) else None)
    _state["last_visual_signal"] = vs_block if isinstance(vs_block, dict) else None
    if _m:
        try:
            _m["visual_signal_state"].set(float(vs_score))
        except Exception as exc:
            logger.debug("visual_signal_state: %s", exc)

    audit_block = plan.get("options_self_audit") if isinstance(plan.get("options_self_audit"), dict) else None
    sa_score = compute_options_self_audit_state_score(audit_block)
    _state["last_options_self_audit"] = dict(audit_block) if isinstance(audit_block, dict) else None
    if _m:
        try:
            _m["self_audit_state"].set(float(sa_score))
        except Exception as exc:
            logger.debug("self_audit_state: %s", exc)

    intent = plan.get("intent") if isinstance(plan.get("intent"), dict) else {}
    entry_allowed = bool(plan.get("entry_allowed"))
    force_no = bool(intent.get("force_no_trade"))
    flags = plan.get("pipeline_quality_flags") or []

    degraded = entry_allowed and isinstance(flags, list) and len(flags) > 4
    if force_no:
        go_val = 0.0
        decision = "force_no_trade"
    elif entry_allowed and not degraded:
        go_val = 1.0
        decision = "go"
    elif entry_allowed and degraded:
        go_val = 0.5
        decision = "go"
    else:
        go_val = 0.0
        decision = "no_go"

    update_options_engine_sentinels()

    if not _m:
        return
    try:
        _m["go_nogo"].set(go_val)
        _m["go_nogo_count"].labels(decision=decision).inc()
    except Exception as exc:
        logger.debug("record_session_plan: %s", exc)


def on_journal_record(journal_path: Path, record: dict[str, Any]) -> None:
    """Tras cada append al JSONL (``record`` completo)."""
    _init_prom()
    _ensure_day()
    event_type = str(record.get("event_type") or "")
    _state["last_journal_path"] = str(journal_path)
    _state["journal_events_today"] = int(_state["journal_events_today"]) + 1
    if event_type == "session_plan":
        _state["journal_sessions_today"] = int(_state["journal_sessions_today"]) + 1
    _state["last_journal_write_ts"] = time.time()
    _state["module_last_ts"]["journal"] = time.time()
    _state["module_status"]["journal"] = 1.0
    if not _m:
        return
    try:
        _m["journal_events"].set(float(_state["journal_events_today"]))
        _m["journal_sessions"].set(float(_state["journal_sessions_today"]))
        _m["journal_write_age"].set(0.0)
        if journal_path.is_file():
            _m["journal_size"].set(float(journal_path.stat().st_size))
        _m["pipe_status"].labels(module="journal").set(1.0)
        _m["pipe_last_run"].labels(module="journal").set(0.0)
    except Exception as exc:
        logger.debug("on_journal_record: %s", exc)


def record_autoclose_triggers(reasons: list[str]) -> None:
    """Incrementa contadores por cada razón de cierre propuesta."""
    _init_prom()
    if not reasons:
        update_options_engine_sentinels()
        return
    if not _m:
        update_options_engine_sentinels()
        return
    try:
        for r in reasons:
            _m["autoclose_triggers"].labels(reason=str(r)).inc()
        _state["module_last_ts"]["autoclose"] = time.time()
        _state["module_status"]["autoclose"] = 1.0
        _m["pipe_status"].labels(module="autoclose").set(1.0)
        _m["pipe_last_run"].labels(module="autoclose").set(0.0)
    except Exception as exc:
        logger.debug("record_autoclose_triggers: %s", exc)
    update_options_engine_sentinels()


def record_options_error(error_type: str) -> None:
    _init_prom()
    if not _m:
        return
    try:
        _m["errors"].labels(type=str(error_type)[:120]).inc()
    except Exception as exc:
        logger.debug("record_options_error: %s", exc)


def refresh_journal_from_disk(path: Path | None = None) -> None:
    """Relee JSONL: conteos del día UTC, trades abiertos, cierres hoy, tamaño, edad escritura, performance paper."""
    _init_prom()
    p = path or Path(_state.get("last_journal_path") or "")
    if not p or not p.is_file():
        if _m:
            try:
                _m["journal_size"].set(0.0)
            except Exception:
                pass
        _apply_paper_performance_gauges(compute_paper_performance_from_pnls([]))
        update_options_engine_sentinels()
        return
    today = _utc_day()
    events_today = 0
    sessions_today = 0
    closed_today = 0
    traces_entry: set[str] = set()
    traces_close: set[str] = set()
    try:
        text = p.read_text(encoding="utf-8")
    except OSError as exc:
        logger.debug("refresh_journal_from_disk read: %s", exc)
        _apply_paper_performance_gauges(compute_paper_performance_from_pnls([]))
        update_options_engine_sentinels()
        return
    close_pnls: list[float] = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        ts = str(row.get("timestamp") or "")[:10]
        et = str(row.get("event_type") or "")
        tid = str(row.get("trace_id") or "")
        if ts == today:
            events_today += 1
            if et == "session_plan":
                sessions_today += 1
            if et == "close_execution":
                closed_today += 1
        if et == "entry_execution" and tid:
            traces_entry.add(tid)
        if et == "close_execution" and tid:
            traces_close.add(tid)
        if et == "close_execution":
            payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
            raw_pnl = payload.get("pnl_realized")
            if raw_pnl is not None:
                try:
                    pv = float(raw_pnl)
                except (TypeError, ValueError):
                    pv = None
                else:
                    if math.isfinite(pv):
                        close_pnls.append(pv)
    _apply_paper_performance_gauges(compute_paper_performance_from_pnls(close_pnls))
    open_cnt = len(traces_entry - traces_close)
    _state["journal_events_today"] = events_today
    _state["journal_sessions_today"] = sessions_today
    _state["paper_closed_today"] = closed_today
    _state["paper_open_trades"] = open_cnt
    _state["last_journal_path"] = str(p)
    try:
        mtime = p.stat().st_mtime
        _state["last_journal_write_ts"] = mtime
    except OSError:
        pass
    if not _m:
        update_options_engine_sentinels()
        return
    try:
        now = time.time()
        lw = float(_state.get("last_journal_write_ts") or 0.0)
        age = max(0.0, now - lw) if lw > 0 else 1e9
        _m["journal_events"].set(float(events_today))
        _m["journal_sessions"].set(float(sessions_today))
        _m["journal_write_age"].set(age)
        _m["journal_size"].set(float(p.stat().st_size))
        _m["paper_open"].set(float(open_cnt))
        _m["paper_closed"].set(float(closed_today))
        _m["paper_phantom"].set(0.0)
        _m["paper_debit_no_stop"].set(0.0)
    except Exception as exc:
        logger.debug("refresh_journal_from_disk gauges: %s", exc)
    update_options_engine_sentinels()


def default_journal_path() -> Path:
    return Path.cwd() / "data" / "options_paper_journal.jsonl"


def get_ui_snapshot() -> dict[str, Any]:
    """Payload para ``/api/options-engine-status`` (hub 8791).

    Alineado con gauges: ``win_rate_approx`` / ``paper_performance_summary`` vienen de
    ``get_last_paper_performance()`` tras ``refresh_journal_from_disk``; ``sentinel_snapshot``
    de ``get_last_sentinel_snapshot()`` (mismas series que paneles H-12…H-16).
    """
    plan = _state.get("last_session_plan")
    if not isinstance(plan, dict):
        plan = {}
    briefing = plan.get("briefing") if isinstance(plan.get("briefing"), dict) else {}
    iv_rank = briefing.get("iv_rank")
    entry_allowed = bool(plan.get("entry_allowed"))
    force_no = bool((plan.get("intent") or {}).get("force_no_trade"))
    if force_no:
        go_label = "NO-GO (force_no_trade)"
    elif entry_allowed:
        qfx = plan.get("pipeline_quality_flags") or []
        go_label = "GO (degraded)" if isinstance(qfx, list) and len(qfx) > 4 else "GO"
    else:
        go_label = "NO-GO"
    target = 100
    jp = Path(_state.get("last_journal_path") or default_journal_path())
    refresh_journal_from_disk(jp)
    closed = int(_state.get("paper_closed_today") or 0)

    perf = get_last_paper_performance()
    win_rate_approx: float | None = None
    paper_performance_summary: dict[str, Any] | None = None
    if isinstance(perf, dict):
        keys = ("n_closes", "win_rate_ratio", "profit_factor", "net_realized_pnl_usd", "max_drawdown_usd")
        paper_performance_summary = {k: perf[k] for k in keys if k in perf}
        wr = perf.get("win_rate_ratio")
        if isinstance(wr, (int, float)) and math.isfinite(float(wr)):
            win_rate_approx = float(wr)

    grafana_url = os.environ.get("ATLAS_GRAFANA_BASE_URL", "http://localhost:3002").rstrip("/")
    dashboard_uid = "atlas-options-health"
    signals_uid = "atlas-options-signals-intent"
    paper_uid = "atlas-options-paper-performance"
    return {
        "ok": True,
        "automation_mode": "paper_only",
        "go_nogo_label": go_label,
        "go_nogo_gauge_hint": (
            0.0
            if force_no
            else (
                0.5
                if entry_allowed
                and len(plan.get("pipeline_quality_flags") or []) > 4
                else (1.0 if entry_allowed else 0.0)
            )
        ),
        "symbol": plan.get("symbol"),
        "iv_rank_current": iv_rank,
        "iv_rank_quality_score": get_last_iv_rank_quality_score(),
        "win_rate_approx": win_rate_approx,
        "paper_performance_summary": paper_performance_summary,
        "paper_trades_closed_today": closed,
        "paper_trades_target": target,
        "paper_trades_progress_pct": min(100.0, 100.0 * closed / target) if target else 0.0,
        "grafana_health_dashboard_url": f"{grafana_url}/d/{dashboard_uid}/options-engine-health",
        "grafana_signals_intent_dashboard_url": f"{grafana_url}/d/{signals_uid}/options-engine-signals-intent",
        "grafana_paper_performance_dashboard_url": f"{grafana_url}/d/{paper_uid}/options-engine-paper-performance",
        "last_updated_utc": datetime.now(timezone.utc).isoformat(),
        "options_self_audit": get_last_options_self_audit(),
        "sentinel_snapshot": get_last_sentinel_snapshot(),
    }
