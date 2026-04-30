"""PolicyManager — persistencia y gestión de políticas activas.

Mantiene el PolicySnapshot en memoria con un TTL configurable
y lo persiste en un archivo JSON para sobrevivir reinicios.

Responsabilidades
-----------------
- Cargar snapshot desde JSON al iniciar
- Aplicar listas de PolicyAction (resultado de run_daily_analysis)
- Exponer snapshot actualizado con caché de 4h por defecto
- Guardar en disco después de cada actualización
- Revertir al snapshot anterior si se detecta degradación
"""
from __future__ import annotations

import json
import logging
import threading
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional

from atlas_code_quant.learning.trade_events import (
    PolicyAction,
    PolicySnapshot,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Serialización / deserialización del snapshot
# ---------------------------------------------------------------------------

def _snapshot_to_dict(s: PolicySnapshot) -> Dict[str, Any]:
    return {
        "enabled_setups": s.enabled_setups,
        "disabled_setups": s.disabled_setups,
        "size_multipliers": s.size_multipliers,
        "score_thresholds": s.score_thresholds,
        "global_score_threshold": s.global_score_threshold,
        "score_boosts": s.score_boosts,
        "manual_review_setups": s.manual_review_setups,
        "last_updated": s.last_updated.isoformat() if s.last_updated else None,
        "last_update_source": s.last_update_source,
    }


def _snapshot_from_dict(d: Dict[str, Any]) -> PolicySnapshot:
    last_updated = None
    if d.get("last_updated"):
        try:
            last_updated = datetime.fromisoformat(d["last_updated"])
        except (ValueError, TypeError):
            pass
    return PolicySnapshot(
        enabled_setups=d.get("enabled_setups", []),
        disabled_setups=d.get("disabled_setups", []),
        size_multipliers=d.get("size_multipliers", {}),
        score_thresholds=d.get("score_thresholds", {}),
        global_score_threshold=d.get("global_score_threshold", 0.40),
        score_boosts=d.get("score_boosts", {}),
        manual_review_setups=d.get("manual_review_setups", []),
        last_updated=last_updated,
        last_update_source=d.get("last_update_source", "loaded"),
    )


# ---------------------------------------------------------------------------
# PolicyManager
# ---------------------------------------------------------------------------

class PolicyManager:
    """Gestiona el ciclo de vida del PolicySnapshot.

    Thread-safe mediante RLock — puede ser llamado desde live_loop y
    desde el runner de análisis diario simultáneamente.

    Args:
        storage_path: ruta al archivo JSON de persistencia
        ttl_hours:    horas de vigencia del caché en memoria (default 4)
    """

    DEFAULT_TTL_HOURS = 4

    def __init__(
        self,
        storage_path: Optional[Path] = None,
        ttl_hours: float = DEFAULT_TTL_HOURS,
    ):
        self._lock = threading.RLock()
        self._ttl = timedelta(hours=ttl_hours)
        self._storage_path = storage_path
        self._snapshot: PolicySnapshot = PolicySnapshot(
            last_updated=datetime.utcnow(),
            last_update_source="init",
        )
        self._previous_snapshot: Optional[PolicySnapshot] = None
        self._loaded_at: datetime = datetime.utcnow()

        if storage_path and storage_path.exists():
            self._load()

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------

    def get_snapshot(self) -> PolicySnapshot:
        """Devuelve el snapshot activo (recarga si expiró el TTL)."""
        with self._lock:
            if self._is_stale():
                self._refresh_from_disk()
            return self._snapshot

    def apply_policies(
        self,
        actions: List[PolicyAction],
        source: str = "daily_analysis",
    ) -> PolicySnapshot:
        """Aplica una lista de PolicyAction y persiste el nuevo snapshot.

        Args:
            actions: lista de PolicyAction propuestas por MetricsEngine
            source:  etiqueta de quién generó las acciones

        Returns:
            nuevo PolicySnapshot aplicado
        """
        with self._lock:
            # Guardar snapshot anterior como respaldo
            self._previous_snapshot = self._snapshot

            new_snap = PolicySnapshot(
                enabled_setups=list(self._snapshot.enabled_setups),
                disabled_setups=list(self._snapshot.disabled_setups),
                size_multipliers=dict(self._snapshot.size_multipliers),
                score_thresholds=dict(self._snapshot.score_thresholds),
                global_score_threshold=self._snapshot.global_score_threshold,
                score_boosts=dict(self._snapshot.score_boosts),
                manual_review_setups=list(self._snapshot.manual_review_setups),
                last_updated=datetime.utcnow(),
                last_update_source=source,
            )

            for action in actions:
                self._apply_single_action(new_snap, action)

            self._snapshot = new_snap
            self._loaded_at = datetime.utcnow()

            if self._storage_path:
                self._save()

            logger.info(
                "PolicyManager: %d acciones aplicadas (source=%s). "
                "Disabled=%s, multipliers=%s",
                len(actions), source,
                new_snap.disabled_setups,
                new_snap.size_multipliers,
            )
            return new_snap

    def revert(self) -> Optional[PolicySnapshot]:
        """Revierte al snapshot anterior (útil si se detecta degradación)."""
        with self._lock:
            if self._previous_snapshot is None:
                logger.warning("PolicyManager.revert: no hay snapshot previo")
                return None
            self._snapshot = self._previous_snapshot
            self._previous_snapshot = None
            self._snapshot.last_update_source = "reverted"
            self._snapshot.last_updated = datetime.utcnow()
            if self._storage_path:
                self._save()
            logger.info("PolicyManager: snapshot revertido al anterior")
            return self._snapshot

    def reset(self) -> PolicySnapshot:
        """Restaura el snapshot a estado inicial (sin restricciones)."""
        with self._lock:
            self._previous_snapshot = self._snapshot
            self._snapshot = PolicySnapshot(
                last_updated=datetime.utcnow(),
                last_update_source="reset",
            )
            if self._storage_path:
                self._save()
            return self._snapshot

    def override(self, snapshot: PolicySnapshot) -> None:
        """Reemplaza el snapshot completo (para uso manual/tests)."""
        with self._lock:
            self._previous_snapshot = self._snapshot
            snapshot.last_updated = datetime.utcnow()
            self._snapshot = snapshot
            self._loaded_at = datetime.utcnow()
            if self._storage_path:
                self._save()

    # ------------------------------------------------------------------
    # Aplicación de acciones individuales
    # ------------------------------------------------------------------

    @staticmethod
    def _apply_single_action(snap: PolicySnapshot, action: PolicyAction) -> None:
        setup = action.setup_type

        if action.action == "disable":
            if setup == "*":
                # Deshabilitar todo no está soportado vía disable — use reset + manual
                logger.warning("PolicyManager: 'disable *' ignorado — use reset()")
            else:
                if setup not in snap.disabled_setups:
                    snap.disabled_setups.append(setup)
                # Quitar de enabled si estaba
                if setup in snap.enabled_setups:
                    snap.enabled_setups.remove(setup)
                snap.size_multipliers[setup] = 0.0

        elif action.action == "enable":
            if setup in snap.disabled_setups:
                snap.disabled_setups.remove(setup)
            if setup not in snap.enabled_setups:
                snap.enabled_setups.append(setup)
            # Restaurar multiplicador si era 0
            if snap.size_multipliers.get(setup, 1.0) == 0.0:
                snap.size_multipliers[setup] = 1.0

        elif action.action == "reduce_size":
            multiplier = action.size_multiplier if action.size_multiplier > 0 else 0.5
            if setup == "*":
                # Reducción global — aplicar a todos los existentes y guardar default
                for k in list(snap.size_multipliers.keys()):
                    snap.size_multipliers[k] *= multiplier
                snap.size_multipliers["__global__"] = multiplier
            else:
                current = snap.size_multipliers.get(setup, 1.0)
                snap.size_multipliers[setup] = round(current * multiplier, 4)

        elif action.action == "increase_priority":
            if setup != "*":
                current_mult = snap.size_multipliers.get(setup, 1.0)
                snap.size_multipliers[setup] = round(
                    min(2.0, current_mult * action.size_multiplier), 4
                )
                if action.score_boost != 0:
                    current_boost = snap.score_boosts.get(setup, 0.0)
                    snap.score_boosts[setup] = round(current_boost + action.score_boost, 4)

        elif action.action == "set_threshold":
            if setup == "*":
                snap.global_score_threshold = max(
                    0.30, min(0.90, action.min_score_threshold)
                )
            else:
                snap.score_thresholds[setup] = max(
                    0.30, min(0.90, action.min_score_threshold)
                )

        if action.score_boost != 0 and action.action not in ("disable", "increase_priority"):
            if setup != "*":
                current = snap.score_boosts.get(setup, 0.0)
                snap.score_boosts[setup] = round(current + action.score_boost, 4)

    # ------------------------------------------------------------------
    # Persistencia
    # ------------------------------------------------------------------

    def _save(self) -> None:
        if not self._storage_path:
            return
        try:
            self._storage_path.parent.mkdir(parents=True, exist_ok=True)
            data = _snapshot_to_dict(self._snapshot)
            tmp = self._storage_path.with_suffix(".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            tmp.replace(self._storage_path)
        except Exception as exc:
            logger.error("PolicyManager._save error: %s", exc)

    def _load(self) -> None:
        try:
            with open(self._storage_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._snapshot = _snapshot_from_dict(data)
            self._loaded_at = datetime.utcnow()
            logger.info(
                "PolicyManager: snapshot cargado desde %s (disabled=%s)",
                self._storage_path, self._snapshot.disabled_setups,
            )
        except Exception as exc:
            logger.error("PolicyManager._load error: %s — usando snapshot vacío", exc)

    def _refresh_from_disk(self) -> None:
        if self._storage_path and self._storage_path.exists():
            self._load()

    def _is_stale(self) -> bool:
        return datetime.utcnow() - self._loaded_at > self._ttl

    # ------------------------------------------------------------------
    # Estado / debug
    # ------------------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        with self._lock:
            snap = self._snapshot
            return {
                "disabled_setups": snap.disabled_setups,
                "size_multipliers": snap.size_multipliers,
                "score_thresholds": snap.score_thresholds,
                "global_score_threshold": snap.global_score_threshold,
                "score_boosts": snap.score_boosts,
                "last_updated": snap.last_updated.isoformat() if snap.last_updated else None,
                "last_update_source": snap.last_update_source,
                "cache_age_minutes": round(
                    (datetime.utcnow() - self._loaded_at).total_seconds() / 60, 1
                ),
                "ttl_hours": self._ttl.total_seconds() / 3600,
            }
