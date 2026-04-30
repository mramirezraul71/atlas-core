"""
Ejecutor HTTP para órdenes multi-leg Tradier (sandbox por defecto).

- Traduce ``LiveOrder`` → formulario ``application/x-www-form-urlencoded``.
- ``dry_run=True``: no abre socket; devuelve el cuerpo que se enviaría.
- ``preview=True`` (con ``dry_run=False``): POST al sandbox con ``preview=true`` (validación sin ejecutar, según docs Tradier).
- Producción (``sandbox=False``) exige ``allow_production=True`` explícito.

No modifica ``AtlasOptionsService`` ni el simulador paper interno.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Literal

from .tradier_live import LiveOrder, LiveOrderLeg

TradierMultilegType = Literal["market", "limit", "debit", "credit", "even"]


def _tradier_side(leg: LiveOrderLeg) -> str:
    """Mapea ``side`` + ``tag`` a buy_to_open / sell_to_open / buy_to_close / sell_to_close."""
    tag = (leg.tag or "open").lower()
    if tag not in ("open", "close"):
        tag = "open"
    if leg.side == "buy":
        return "buy_to_open" if tag == "open" else "buy_to_close"
    return "sell_to_open" if tag == "open" else "sell_to_close"


def _build_form_body(
    order: LiveOrder,
    *,
    preview: bool,
    multileg_type: TradierMultilegType,
    duration: Literal["day", "gtc"],
) -> list[tuple[str, str]]:
    pairs: list[tuple[str, str]] = [
        ("class", "multileg"),
        ("symbol", order.symbol),
        ("duration", duration),
        ("type", multileg_type),
    ]
    if preview:
        pairs.append(("preview", "true"))
    for leg in order.legs:
        i = leg.leg_index
        pairs.append((f"side[{i}]", _tradier_side(leg)))
        pairs.append((f"quantity[{i}]", str(int(leg.quantity))))
        pairs.append((f"option_symbol[{i}]", leg.contract_symbol))
    if multileg_type == "limit" and order.legs:
        # Net limit aproximado: suma algebraica según buy/sell (USD por spread); ajustar en capas superiores si hace falta.
        net = 0.0
        for leg in order.legs:
            px = float(leg.limit_price or 0.0)
            sign = 1.0 if leg.side == "buy" else -1.0
            net += sign * px * int(leg.quantity) * 100
        pairs.append(("price", f"{abs(net):.2f}"))
    return pairs


@dataclass(kw_only=True)
class TradierOrderExecutor:
    """
    Cliente mínimo POST ``/v1/accounts/{account_id}/orders``.

    Variables de entorno recomendadas para tokens (no leer archivos de credenciales desde aquí).
    """

    account_id: str
    token: str
    sandbox: bool = True
    timeout: int = 10
    dry_run: bool = False
    allow_production: bool = False
    _last_request_form: list[tuple[str, str]] = field(default_factory=list, repr=False)
    _last_response: dict[str, Any] | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if not self.sandbox and not self.allow_production:
            raise ValueError(
                "API de producción deshabilitada: pasa sandbox=True o allow_production=True "
                "solo si aceptas órdenes en cuenta real."
            )

    @property
    def base_url(self) -> str:
        if self.sandbox:
            return "https://sandbox.tradier.com/v1"
        return "https://api.tradier.com/v1"

    def place_multileg_order(
        self,
        order: LiveOrder,
        *,
        preview: bool = True,
        multileg_type: TradierMultilegType | None = None,
        duration: Literal["day", "gtc"] = "day",
    ) -> dict[str, Any]:
        """
        Envía (o simula) la orden multi-leg.

        - ``multileg_type``: si es ``None``, usa ``market`` salvo que todas las patas sean ``limit``,
          en cuyo caso usa ``limit`` y un ``price`` neto aproximado.
        - ``preview``: añade ``preview=true`` al formulario (validación Tradier).
        """
        if not order.legs:
            raise ValueError("LiveOrder sin legs")
        if not order.symbol:
            raise ValueError("LiveOrder.symbol (subyacente) requerido")

        mtype: TradierMultilegType
        if multileg_type is not None:
            mtype = multileg_type
        elif all(lg.order_type == "limit" for lg in order.legs):
            mtype = "limit"
        else:
            mtype = "market"

        form_pairs = _build_form_body(
            order,
            preview=preview,
            multileg_type=mtype,
            duration=duration,
        )
        self._last_request_form = list(form_pairs)
        body_bytes = urllib.parse.urlencode(form_pairs).encode("utf-8")
        url = f"{self.base_url}/accounts/{self.account_id}/orders"

        if self.dry_run:
            out: dict[str, Any] = {
                "dry_run": True,
                "preview": preview,
                "url": url,
                "method": "POST",
                "content_type": "application/x-www-form-urlencoded",
                "form": dict(form_pairs),
                "multileg_type": mtype,
            }
            self._last_response = out
            return out

        req = urllib.request.Request(
            url,
            data=body_bytes,
            method="POST",
            headers={
                "Authorization": f"Bearer {self.token}",
                "Accept": "application/json",
                "Content-Type": "application/x-www-form-urlencoded",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                status = getattr(resp, "status", 200)
        except urllib.error.HTTPError as e:
            try:
                err_body = e.read().decode("utf-8", errors="replace")
            except OSError:
                err_body = str(e)
            parsed = _try_json(err_body)
            result = {
                "ok": False,
                "http_status": e.code,
                "preview": preview,
                "sandbox": self.sandbox,
                "body": parsed if parsed is not None else err_body,
                "error": "HTTPError",
                "multileg_type": mtype,
            }
            self._last_response = result
            return result
        except urllib.error.URLError as e:
            reason = getattr(e, "reason", e)
            result = {
                "ok": False,
                "http_status": None,
                "preview": preview,
                "sandbox": self.sandbox,
                "body": None,
                "error": str(reason),
                "multileg_type": mtype,
            }
            self._last_response = result
            return result

        parsed = _try_json(raw)
        result = {
            "ok": 200 <= status < 300,
            "http_status": status,
            "preview": preview,
            "sandbox": self.sandbox,
            "body": parsed if parsed is not None else raw,
            "multileg_type": mtype,
        }
        self._last_response = result
        return result

    @property
    def last_response(self) -> dict[str, Any] | None:
        return self._last_response


def _try_json(s: str) -> Any | None:
    try:
        return json.loads(s)
    except json.JSONDecodeError:
        return None
