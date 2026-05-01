"""Render de mensajes: HTML (Telegram) y texto plano (WhatsApp)."""
from __future__ import annotations

import html
from typing import Any

from notifications.models import PrioritizedBriefing


def _esc(s: Any) -> str:
    return html.escape(str(s), quote=True)


def render_premarket(b: PrioritizedBriefing, *, mode: str = "telegram_html") -> tuple[str, str]:
    lines_html: list[str] = [
        f"<b>{_esc(b.headline)}</b>",
        "",
        "<b>Estado</b>",
        f"• Readiness: {'<b>OK</b>' if b.metrics.get('readiness_ready') else '<b>NO LISTO</b>'}",
        f"• Scanner corriendo: {b.metrics.get('scanner_running')}",
        f"• Posiciones abiertas: {b.metrics.get('open_positions')}",
        "",
        "<b>Aprendizaje</b>",
        f"• Fase: <code>{_esc(b.learning_summary.get('phase'))}</code>",
        f"• Política puede pesar: {b.learning_summary.get('policy_can_weight')}",
    ]
    ic = b.learning_summary.get("orchestrator") or {}
    if ic.get("last_ic_by_method"):
        lines_html.append(f"• IC por método: <code>{_esc(str(ic.get('last_ic_by_method'))[:180])}</code>")
    ad = b.learning_summary.get("adaptive") or {}
    lines_html.append(f"• Adaptive: {_esc(ad.get('headline'))}")
    lines_html += ["", "<b>Oportunidades priorizadas</b>"]
    if not b.opportunities:
        lines_html.append("<i>Ninguna por encima del umbral actual.</i>")
    for o in b.opportunities:
        lines_html.append(
            f"• <b>{_esc(o.get('symbol'))}</b> score={o.get('selection_score')} "
            f"tf={_esc(o.get('timeframe'))} m={_esc(o.get('method'))}"
        )
    lines_html += ["", "<b>Riesgos / foco</b>"]
    for r in b.risks:
        lines_html.append(f"• {_esc(r)}")
    lines_html += ["", "<b>Posiciones a vigilar</b>"]
    if not b.positions_focus:
        lines_html.append("<i>Sin posiciones o sin datos.</i>")
    for p in b.positions_focus:
        lines_html.append(
            f"• <b>{_esc(p.get('symbol'))}</b> uPnL={_esc(p.get('unrealized_pnl'))} "
            f"type={_esc(p.get('strategy_type'))}"
        )
    lines_html += ["", "<b>Plan de sesión (propuesta)</b>"]
    for i, step in enumerate(b.session_plan, 1):
        lines_html.append(f"{i}. {_esc(step)}")
    oc = b.open_close_criteria
    if oc:
        lines_html += ["", "<b>Criterios apertura</b>"]
        for x in oc.get("open") or []:
            lines_html.append(f"• {_esc(x)}")
        lines_html += ["", "<b>Criterios cierre</b>"]
        for x in oc.get("close") or []:
            lines_html.append(f"• {_esc(x)}")
    body_html = "\n".join(lines_html)
    plain = _html_to_plain(body_html)
    return body_html, plain


def render_eod(b: PrioritizedBriefing, *, mode: str = "telegram_html") -> tuple[str, str]:
    lines_html = [
        f"<b>{_esc(b.headline)}</b>",
        "",
        f"• PnL día (estim. desde journal): <code>{_esc(b.metrics.get('pnl_day_estimate'))}</code>",
        f"• Filas analytics: {_esc(b.metrics.get('trades_analytics_rows'))}",
        "",
        "<b>Aprendizaje</b>",
        f"• Fase: <code>{_esc(b.learning_summary.get('phase'))}</code>",
    ]
    ad = b.learning_summary.get("adaptive") or {}
    lines_html.append(f"• Adaptive: {_esc(ad.get('headline'))}")
    lines_html += ["", "<b>Riesgos residuales</b>"]
    for r in b.risks:
        lines_html.append(f"• {_esc(r)}")
    lines_html += ["", "<b>Recomendaciones próxima sesión</b>"]
    for step in b.session_plan:
        lines_html.append(f"• {_esc(step)}")
    body_html = "\n".join(lines_html)
    plain = _html_to_plain(body_html)
    return body_html, plain


def render_exit_intelligence(
    *,
    symbol: str,
    title_suffix: str,
    body_lines: list[str],
) -> tuple[str, str]:
    lines_html = [f"<b>Exit intelligence — {_esc(symbol)}</b>", _esc(title_suffix), ""] + [
        _esc(line) for line in body_lines
    ]
    body_html = "\n".join(lines_html)
    return body_html, _html_to_plain(body_html)


def render_intraday(title: str, lines: list[str]) -> tuple[str, str]:
    body_html = f"<b>{_esc(title)}</b>\n" + "\n".join(_esc(x) for x in lines)
    return body_html, _html_to_plain(body_html)


def _html_to_plain(html_text: str) -> str:
    import re

    t = re.sub(r"<br\s*/?>", "\n", html_text, flags=re.I)
    t = re.sub(r"</p>\s*", "\n", t, flags=re.I)
    t = re.sub(r"<[^>]+>", "", t)
    t = (
        t.replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&amp;", "&")
        .replace("&quot;", '"')
    )
    return t.strip()
