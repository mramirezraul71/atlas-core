"""IC Signal Tracker — Information Coefficient tracking per scanner method.

Mide la calidad predictiva real de las señales del scanner usando el
Information Coefficient (IC) = correlación de rango (Spearman) entre
la predicción de movimiento (predicted_move_pct) y el retorno real
observado N días después.

Marco teórico:
- Grinold & Kahn, "Active Portfolio Management" (2000), Cap. 6:
    IR ≈ IC × √Breadth
    IC > 0.05 considerado meaningful; IC > 0.10 es fuerte.
    t-stat(IC) ≥ 2.0 requerido antes de usar una señal en producción.
- Asness, Moskowitz & Pedersen, "Value and Momentum Everywhere" (2013):
    Valida metodología de IC en múltiples activos y horizontes.

Uso:
    tracker = ICSignalTracker()
    tracker.record_signal(symbol="AAPL", method="trend_ema_stack",
                          predicted_move_pct=2.5, entry_price=175.0,
                          timeframe="1d", selection_score=0.82)
    # ... días después, cuando el precio está disponible:
    tracker.update_outcome(symbol="AAPL", signal_id="...", exit_price=178.2)
    summary = tracker.summary()
    # summary["by_method"]["trend_ema_stack"]["ic"] => 0.07
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_DEFAULT_TRACKER_PATH = (
    Path(__file__).resolve().parent.parent / "data" / "learning" / "ic_tracker.json"
)
_MIN_SAMPLE_FOR_IC = 5   # mínimo para calcular IC (prudente, no definitivo)
_MIN_SAMPLE_MEANINGFUL = 30  # mínimo Grinold-Kahn para señal "useful"


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
        return result if result == result else default  # NaN check
    except (TypeError, ValueError):
        return default


def _parse_iso_utc(value: Any) -> datetime | None:
    raw = str(value or "").strip()
    if not raw:
        return None
    try:
        parsed = datetime.fromisoformat(raw.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _spearman_r(x: list[float], y: list[float]) -> tuple[float, float]:
    """Correlación de Spearman e t-estadístico.

    Retorna (rho, t_stat). Si n < 2 retorna (0.0, 0.0).
    Implementación manual para no requerir scipy en runtime básico.
    """
    n = len(x)
    if n < 2 or len(y) != n:
        return 0.0, 0.0
    try:
        # Calcular rangos
        def _rank(v: list[float]) -> list[float]:
            sorted_idx = sorted(range(n), key=lambda i: v[i])
            ranks = [0.0] * n
            i = 0
            while i < n:
                j = i
                while j < n - 1 and v[sorted_idx[j]] == v[sorted_idx[j + 1]]:
                    j += 1
                avg_rank = (i + j) / 2.0 + 1.0
                for k in range(i, j + 1):
                    ranks[sorted_idx[k]] = avg_rank
                i = j + 1
            return ranks

        rx = _rank(x)
        ry = _rank(y)
        # Pearson sobre rangos = Spearman
        mean_rx = sum(rx) / n
        mean_ry = sum(ry) / n
        num = sum((rx[i] - mean_rx) * (ry[i] - mean_ry) for i in range(n))
        den_x = sum((rx[i] - mean_rx) ** 2 for i in range(n)) ** 0.5
        den_y = sum((ry[i] - mean_ry) ** 2 for i in range(n)) ** 0.5
        if den_x == 0 or den_y == 0:
            return 0.0, 0.0
        rho = num / (den_x * den_y)
        rho = max(-1.0, min(1.0, rho))
        # t-stat = rho * sqrt((n-2)/(1-rho²))
        if abs(rho) >= 1.0:
            t_stat = float("inf") if rho > 0 else float("-inf")
        else:
            t_stat = rho * ((n - 2) / max(1e-12, 1 - rho ** 2)) ** 0.5
        return round(rho, 6), round(t_stat, 4)
    except Exception:
        return 0.0, 0.0


class ICSignalTracker:
    """Registra señales del scanner y mide su IC vs. retorno real.

    Estado persistido en data/learning/ic_tracker.json.
    No es un archivo de runtime operativo: puede versionarse si se desea,
    aunque no se incluye en commits de estado por defecto.
    """

    def __init__(self, tracker_path: Path | None = None) -> None:
        self._path = Path(tracker_path) if tracker_path else _DEFAULT_TRACKER_PATH
        self._state: dict[str, Any] = self._load()

    # ── Persistencia ──────────────────────────────────────────────────────────

    def _load(self) -> dict[str, Any]:
        if not self._path.exists():
            return {"signals": {}, "updated_at": _utcnow_iso()}
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return data
        except (OSError, json.JSONDecodeError):
            pass
        return {"signals": {}, "updated_at": _utcnow_iso()}

    def _save(self) -> None:
        self._state["updated_at"] = _utcnow_iso()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(
            json.dumps(self._state, indent=2, default=str), encoding="utf-8"
        )

    # ── API pública ───────────────────────────────────────────────────────────

    def record_signal(
        self,
        *,
        symbol: str,
        method: str,
        predicted_move_pct: float,
        entry_price: float,
        timeframe: str = "1d",
        selection_score: float = 0.0,
        signal_id: str | None = None,
    ) -> str:
        """Registra una señal del scanner al momento de evaluación.

        Args:
            symbol: Ticker evaluado.
            method: Método del scanner (trend_ema_stack, breakout_donchian, etc.).
            predicted_move_pct: Movimiento esperado en % (signed: + alcista, - bajista).
            entry_price: Precio de referencia al disparo de señal.
            timeframe: Marco temporal de la señal.
            selection_score: Score de selección del scanner (0-1).
            signal_id: ID único opcional; se genera UUID si None.

        Returns:
            signal_id asignado.
        """
        sid = signal_id or str(uuid.uuid4())
        self._state.setdefault("signals", {})[sid] = {
            "signal_id": sid,
            "symbol": str(symbol).upper(),
            "method": str(method),
            "predicted_move_pct": _safe_float(predicted_move_pct),
            "entry_price": _safe_float(entry_price),
            "timeframe": str(timeframe),
            "selection_score": _safe_float(selection_score),
            "recorded_at": _utcnow_iso(),
            "actual_return_pct": None,
            "measured_at": None,
            "outcome_available": False,
        }
        self._save()
        return sid

    def update_outcome(
        self,
        *,
        signal_id: str,
        exit_price: float,
    ) -> bool:
        """Registra el retorno real para una señal previamente grabada.

        Args:
            signal_id: ID devuelto por record_signal().
            exit_price: Precio de salida (cierre de posición o precio N días después).

        Returns:
            True si se actualizó, False si no se encontró la señal.
        """
        signals = self._state.get("signals") or {}
        if signal_id not in signals:
            return False
        sig = signals[signal_id]
        entry = _safe_float(sig.get("entry_price"), 0.0)
        if entry <= 0:
            return False
        actual_return_pct = round(((exit_price - entry) / entry) * 100.0, 6)
        sig["actual_return_pct"] = actual_return_pct
        sig["measured_at"] = _utcnow_iso()
        sig["outcome_available"] = True
        self._save()
        return True

    def find_pending_signal_id(
        self,
        *,
        symbol: str,
        method: str | None = None,
        entry_price: float | None = None,
        recorded_near: str | None = None,
    ) -> str | None:
        """Encuentra la señal pendiente más probable para una posición cerrada.

        Se prioriza por cercanía temporal al momento de entrada y, como apoyo,
        por cercanía del precio de entrada. Esto permite conectar outcomes del
        journal aunque el signal_id no haya viajado por todo el pipeline.
        """
        target_symbol = str(symbol or "").upper().strip()
        if not target_symbol:
            return None

        target_method = str(method or "").strip() or None
        target_entry_price = abs(_safe_float(entry_price, 0.0))
        target_recorded_at = _parse_iso_utc(recorded_near)

        candidates: list[tuple[float, str]] = []
        signals = self._state.get("signals") or {}
        for signal_id, sig in signals.items():
            if sig.get("outcome_available"):
                continue
            if str(sig.get("symbol") or "").upper().strip() != target_symbol:
                continue
            if target_method and str(sig.get("method") or "").strip() != target_method:
                continue

            recorded_at = _parse_iso_utc(sig.get("recorded_at"))
            time_penalty = 0.0
            if target_recorded_at and recorded_at:
                time_penalty = abs((recorded_at - target_recorded_at).total_seconds())
            elif target_recorded_at or recorded_at:
                time_penalty = 10**9

            price_penalty = 0.0
            signal_entry = abs(_safe_float(sig.get("entry_price"), 0.0))
            if target_entry_price > 0 and signal_entry > 0:
                price_penalty = abs(signal_entry - target_entry_price)
            elif target_entry_price > 0 or signal_entry > 0:
                price_penalty = 10**6

            candidates.append((time_penalty + price_penalty, signal_id))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def update_pending_outcome(
        self,
        *,
        symbol: str,
        method: str,
        entry_price: float,
        recorded_near: str,
        exit_price: float,
    ) -> str | None:
        """Busca la señal pendiente más cercana y registra su outcome.

        Útil cuando el cierre de posición no tiene signal_id pero sí tiene el contexto
        original (símbolo, método, precio de entrada, timestamp aproximado).

        Matching: filtra por symbol+method+outcome_available=False, luego elige la señal
        con |entry_price_diff| mínimo; en caso de empate, la más cercana en tiempo a
        recorded_near.

        Args:
            symbol: Ticker.
            method: Método del scanner.
            entry_price: Precio de entrada de la señal original.
            recorded_near: Timestamp ISO aproximado del momento de la señal.
            exit_price: Precio de salida para calcular el retorno real.

        Returns:
            signal_id actualizado, o None si no se encontró candidato.
        """
        signals = self._state.get("signals") or {}
        sym_upper = str(symbol).upper()
        try:
            target_dt = datetime.fromisoformat(
                str(recorded_near).replace("Z", "+00:00")
            )
            if target_dt.tzinfo is None:
                target_dt = target_dt.replace(tzinfo=timezone.utc)
        except ValueError:
            target_dt = None

        candidates = [
            sig for sig in signals.values()
            if not sig.get("outcome_available")
            and str(sig.get("symbol") or "").upper() == sym_upper
            and str(sig.get("method") or "") == str(method)
        ]
        if not candidates:
            return None

        def _sort_key(sig: dict) -> tuple[float, float]:
            price_diff = abs(_safe_float(sig.get("entry_price"), 0.0) - _safe_float(entry_price, 0.0))
            time_diff = 0.0
            if target_dt is not None:
                try:
                    rec = datetime.fromisoformat(
                        str(sig.get("recorded_at", "")).replace("Z", "+00:00")
                    )
                    if rec.tzinfo is None:
                        rec = rec.replace(tzinfo=timezone.utc)
                    time_diff = abs((target_dt - rec).total_seconds())
                except ValueError:
                    time_diff = float("inf")
            return (price_diff, time_diff)

        best = min(candidates, key=_sort_key)
        updated = self.update_outcome(signal_id=best["signal_id"], exit_price=exit_price)
        return best["signal_id"] if updated else None

    def compute_ic(
        self,
        method: str | None = None,
        min_sample: int = _MIN_SAMPLE_FOR_IC,
    ) -> dict[str, Any]:
        """Calcula IC para un método específico o para todos los métodos.

        Args:
            method: Nombre del método. None = todos combinados.
            min_sample: Mínimo de observaciones para calcular IC.

        Returns:
            Dict con ic, t_stat, n_observations, interpretation, ic_status.
        """
        signals = self._state.get("signals") or {}
        completed = [
            s for s in signals.values()
            if s.get("outcome_available")
            and s.get("predicted_move_pct") is not None
            and s.get("actual_return_pct") is not None
            and (method is None or str(s.get("method") or "") == method)
        ]
        n = len(completed)
        if n < min_sample:
            return {
                "ic": None,
                "t_stat": None,
                "n_observations": n,
                "min_sample_required": min_sample,
                "ic_status": "insufficient_data",
                "interpretation": (
                    f"Solo {n} señales con outcome disponible "
                    f"(mínimo {min_sample} para calcular IC). "
                    "Continuar operando en paper para acumular muestra."
                ),
            }
        predicted = [_safe_float(s["predicted_move_pct"]) for s in completed]
        actual = [_safe_float(s["actual_return_pct"]) for s in completed]
        rho, t_stat = _spearman_r(predicted, actual)
        ic_status = _ic_status_label(rho, t_stat, n)
        return {
            "ic": rho,
            "t_stat": t_stat,
            "n_observations": n,
            "min_sample_required": min_sample,
            "min_sample_meaningful": _MIN_SAMPLE_MEANINGFUL,
            "ic_status": ic_status,
            "interpretation": _ic_interpretation(rho, t_stat, n),
        }

    def summary(self) -> dict[str, Any]:
        """Resumen completo: IC global y desglose por método.

        Returns:
            Dict con overall IC, breakdown by method, tracker stats.
        """
        signals = self._state.get("signals") or {}
        total = len(signals)
        with_outcome = sum(1 for s in signals.values() if s.get("outcome_available"))
        pending_outcome = total - with_outcome

        # Desglose por método
        methods: set[str] = {str(s.get("method") or "unknown") for s in signals.values()}
        by_method: dict[str, Any] = {}
        for m in sorted(methods):
            by_method[m] = self.compute_ic(method=m)

        # IC global
        overall = self.compute_ic(method=None)

        return {
            "generated_at": _utcnow_iso(),
            "total_signals": total,
            "signals_with_outcome": with_outcome,
            "signals_pending_outcome": pending_outcome,
            "overall": overall,
            "by_method": by_method,
            "benchmark_reference": {
                "source": "Grinold & Kahn — Active Portfolio Management (2000), Cap. 6",
                "ic_threshold_meaningful": 0.05,
                "ic_threshold_strong": 0.10,
                "t_stat_threshold": 2.0,
                "min_sample_production": _MIN_SAMPLE_MEANINGFUL,
            },
        }

    def pending_outcome_signals(
        self, *, older_than_hours: float = 0.0
    ) -> list[dict[str, Any]]:
        """Lista señales sin outcome, opcionalmente filtradas por antigüedad.

        Útil para el loop autónomo: después de N horas, buscar el precio actual
        y llamar update_outcome().

        Args:
            older_than_hours: Retorna solo señales grabadas hace más de N horas.

        Returns:
            Lista de dicts con signal_id, symbol, entry_price, recorded_at.
        """
        signals = self._state.get("signals") or {}
        now = datetime.now(timezone.utc)
        result = []
        for sig in signals.values():
            if sig.get("outcome_available"):
                continue
            if older_than_hours > 0:
                recorded = _parse_iso_utc(sig.get("recorded_at"))
                if recorded is not None:
                    age_h = (now - recorded).total_seconds() / 3600.0
                    if age_h < older_than_hours:
                        continue
                else:
                    pass
            result.append({
                "signal_id": sig["signal_id"],
                "symbol": sig["symbol"],
                "method": sig["method"],
                "entry_price": sig["entry_price"],
                "predicted_move_pct": sig["predicted_move_pct"],
                "recorded_at": sig["recorded_at"],
            })
        return result

    def purge_old_signals(self, *, keep_days: int = 90) -> int:
        """Elimina señales antiguas para mantener el archivo manejable.

        Args:
            keep_days: Conserva señales de los últimos N días.

        Returns:
            Número de señales eliminadas.
        """
        signals = self._state.get("signals") or {}
        now = datetime.now(timezone.utc)
        to_delete = []
        for sid, sig in signals.items():
            try:
                recorded = _parse_iso_utc(sig.get("recorded_at"))
                if recorded is None:
                    raise ValueError("invalid recorded_at")
                age_days = (now - recorded).total_seconds() / 86400.0
                if age_days > keep_days:
                    to_delete.append(sid)
            except (ValueError, KeyError):
                to_delete.append(sid)
        for sid in to_delete:
            del signals[sid]
        if to_delete:
            self._save()
        return len(to_delete)


# ── Helpers de interpretación ──────────────────────────────────────────────────

def _ic_status_label(rho: float, t_stat: float, n: int) -> str:
    """Clasifica el IC según benchmarks Grinold-Kahn."""
    if n < _MIN_SAMPLE_FOR_IC:
        return "insufficient_data"
    if abs(t_stat) < 2.0:
        return "not_significant"
    if rho < 0:
        return "negative"
    if rho < 0.05:
        return "weak"
    if rho < 0.10:
        return "meaningful"
    return "strong"


def _ic_interpretation(rho: float, t_stat: float, n: int) -> str:
    """Interpretación textual del IC para el scorecard."""
    if n < _MIN_SAMPLE_FOR_IC:
        return (
            f"Muestra insuficiente ({n} señales). "
            "Se necesitan al menos 5 para calcular IC, 30 para señal útil."
        )
    sig_str = "significativo (t≥2)" if abs(t_stat) >= 2.0 else "NO significativo (t<2)"
    if rho < 0:
        return (
            f"IC negativo ({rho:.4f}, t={t_stat:.2f}, {sig_str}): "
            "la señal predice en dirección contraria al resultado. "
            "Revisar criterio de dirección o filtro de régimen."
        )
    if abs(t_stat) < 2.0:
        return (
            f"IC={rho:.4f} pero t={t_stat:.2f} (no significativo con n={n}). "
            f"Necesita al menos {_MIN_SAMPLE_MEANINGFUL} observaciones para evaluar."
        )
    if rho < 0.05:
        return (
            f"IC débil ({rho:.4f}, t={t_stat:.2f}, n={n}). "
            "Por debajo del umbral meaningful (0.05) de Grinold-Kahn."
        )
    if rho < 0.10:
        return (
            f"IC meaningful ({rho:.4f}, t={t_stat:.2f}, n={n}). "
            "Supera umbral 0.05 de Grinold-Kahn. Continuar acumulando muestra."
        )
    return (
        f"IC fuerte ({rho:.4f}, t={t_stat:.2f}, n={n}). "
        "Supera umbral 0.10 de Grinold-Kahn. Señal con poder predictivo real."
    )


# ── Singleton para uso en API ──────────────────────────────────────────────────

_TRACKER_INSTANCE: ICSignalTracker | None = None


def get_ic_tracker(tracker_path: Path | None = None) -> ICSignalTracker:
    """Retorna la instancia singleton del tracker."""
    global _TRACKER_INSTANCE
    if _TRACKER_INSTANCE is None:
        _TRACKER_INSTANCE = ICSignalTracker(tracker_path=tracker_path)
    return _TRACKER_INSTANCE
