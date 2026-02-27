"""
OutcomePredictor: Predice resultados de acciones basandose en historial.

Usa estadisticas de action_outcomes del WorldModel y opcionalmente
consulta memorias episodicas para enriquecer la prediccion.
"""
from __future__ import annotations

import json
import sqlite3
from typing import Any, Dict, List, Optional

from .engine import _con


class OutcomePredictor:
    """Predice outcomes de acciones usando historial + estadisticas bayesianas."""

    def predict(self, action_type: str, context: Dict[str, Any] = None) -> Dict[str, Any]:
        stats = self._get_stats(action_type)
        similar = self._find_similar_contexts(action_type, context or {})

        if stats["total"] == 0 and not similar:
            return {
                "action_type": action_type,
                "prediction": "unknown",
                "confidence": 0.0,
                "reason": "sin historial previo",
                "recommendation": "ejecutar con precaucion",
            }

        base_rate = stats["success_rate"]
        context_rate = self._context_success_rate(similar) if similar else base_rate

        blended = 0.4 * base_rate + 0.6 * context_rate if similar else base_rate
        confidence = min(1.0, stats["total"] / 20.0)

        if blended >= 0.8:
            prediction = "success"
            recommendation = "alta probabilidad de exito"
        elif blended >= 0.5:
            prediction = "partial"
            recommendation = "resultado mixto esperado, considerar alternativas"
        else:
            prediction = "failure"
            recommendation = "alto riesgo de fallo, buscar estrategia alternativa"

        return {
            "action_type": action_type,
            "prediction": prediction,
            "success_probability": round(blended, 3),
            "confidence": round(confidence, 3),
            "base_success_rate": round(base_rate, 3),
            "context_success_rate": round(context_rate, 3) if similar else None,
            "similar_experiences": len(similar),
            "total_history": stats["total"],
            "avg_duration_ms": stats["avg_duration_ms"],
            "recommendation": recommendation,
        }

    def suggest_alternative(self, action_type: str) -> Optional[Dict[str, Any]]:
        """Sugiere accion alternativa con mejor tasa de exito."""
        with _con() as c:
            rows = c.execute("""
                SELECT action_type, COUNT(*) as n,
                       AVG(success) as rate, AVG(reward) as avg_r
                FROM action_outcomes
                WHERE action_type != ?
                GROUP BY action_type
                HAVING n >= 3
                ORDER BY rate DESC, avg_r DESC
                LIMIT 5
            """, (action_type,)).fetchall()

        if not rows:
            return None

        return {
            "alternatives": [
                {"action": r["action_type"], "success_rate": round(r["rate"], 3),
                 "avg_reward": round(r["avg_r"], 3), "count": r["n"]}
                for r in rows
            ]
        }

    def _get_stats(self, action_type: str) -> Dict[str, Any]:
        with _con() as c:
            row = c.execute("""
                SELECT COUNT(*) as total, COALESCE(AVG(success), 0) as success_rate,
                       COALESCE(AVG(reward), 0) as avg_reward,
                       COALESCE(AVG(duration_ms), 0) as avg_duration_ms
                FROM action_outcomes WHERE action_type = ?
            """, (action_type,)).fetchone()
            return dict(row)

    def _find_similar_contexts(self, action_type: str, context: Dict[str, Any],
                                limit: int = 10) -> List[Dict]:
        if not context:
            return []
        with _con() as c:
            rows = c.execute("""
                SELECT context, success, reward FROM action_outcomes
                WHERE action_type = ? ORDER BY timestamp_ts DESC LIMIT ?
            """, (action_type, limit * 3)).fetchall()

        scored = []
        for r in rows:
            try:
                ctx = json.loads(r["context"] or "{}")
            except Exception:
                continue
            overlap = sum(1 for k, v in context.items() if ctx.get(k) == v)
            if overlap > 0:
                scored.append({
                    "context": ctx, "success": r["success"],
                    "reward": r["reward"], "overlap": overlap
                })
        scored.sort(key=lambda x: x["overlap"], reverse=True)
        return scored[:limit]

    def _context_success_rate(self, similar: List[Dict]) -> float:
        if not similar:
            return 0.5
        total_weight = sum(s["overlap"] for s in similar)
        if total_weight == 0:
            return 0.5
        weighted = sum(s["success"] * s["overlap"] for s in similar)
        return weighted / total_weight
