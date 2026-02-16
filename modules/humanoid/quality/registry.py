"""
POT Registry: Búsqueda y selección de POTs.
"""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from .models import POT, POTCategory

_log = logging.getLogger("humanoid.quality.registry")


def _get_all_pots() -> List[POT]:
    """Carga todos los POTs del registry."""
    from .pots import _POT_MODULES
    pots = []
    for pot_id, factory in _POT_MODULES.items():
        try:
            pots.append(factory())
        except Exception as e:
            _log.warning("Failed to load POT %s: %s", pot_id, e)
    return pots


def list_pots(
    category: Optional[str] = None,
    severity: Optional[str] = None,
    tags: Optional[List[str]] = None,
) -> List[Dict[str, Any]]:
    """
    Lista POTs disponibles con filtros opcionales.
    
    Args:
        category: Filtrar por categoría (repair, maintenance, incident, etc)
        severity: Filtrar por severidad (low, medium, high, critical)
        tags: Filtrar por tags (debe contener todos los especificados)
    
    Returns:
        Lista de diccionarios con info resumida de cada POT
    """
    pots = _get_all_pots()
    results = []
    
    for pot in pots:
        # Filtro por categoría
        if category:
            pot_cat = pot.category.value if isinstance(pot.category, POTCategory) else pot.category
            if pot_cat != category:
                continue
        
        # Filtro por severidad
        if severity:
            pot_sev = pot.severity.value if hasattr(pot.severity, 'value') else pot.severity
            if pot_sev != severity:
                continue
        
        # Filtro por tags
        if tags:
            if not all(t in pot.tags for t in tags):
                continue
        
        results.append({
            "id": pot.id,
            "name": pot.name,
            "category": pot.category.value if isinstance(pot.category, POTCategory) else pot.category,
            "severity": pot.severity.value if hasattr(pot.severity, 'value') else pot.severity,
            "description": pot.description[:200],
            "steps_count": len(pot.steps),
            "estimated_minutes": pot.estimated_duration_minutes,
            "tags": pot.tags,
            "trigger_check_ids": pot.trigger_check_ids,
        })
    
    return results


def get_pot(pot_id: str) -> Optional[POT]:
    """
    Obtiene un POT por su ID.
    
    Args:
        pot_id: Identificador del POT (ej: "camera_repair")
    
    Returns:
        Instancia del POT o None si no existe
    """
    from .pots import get_pot_factory
    factory = get_pot_factory(pot_id)
    if not factory:
        return None
    try:
        return factory()
    except Exception as e:
        _log.error("Failed to instantiate POT %s: %s", pot_id, e)
        return None


def get_pot_by_incident(
    check_id: str,
    message: str = "",
    severity: Optional[str] = None,
) -> Optional[POT]:
    """
    Selecciona el POT más apropiado para un incidente dado.
    
    Usa el check_id y mensaje para hacer matching con trigger_check_ids y trigger_keywords.
    
    Args:
        check_id: ID del check que generó el incidente
        message: Mensaje del incidente
        severity: Severidad del incidente (opcional, para refinar selección)
    
    Returns:
        El POT más apropiado o None si no hay match
    """
    pots = _get_all_pots()
    check_lower = (check_id or "").lower()
    msg_lower = (message or "").lower()
    
    # Score de cada POT
    scores: List[tuple] = []
    
    for pot in pots:
        score = 0
        
        # Match por trigger_check_ids
        for trigger in pot.trigger_check_ids:
            trigger_lower = trigger.lower()
            if trigger_lower.endswith("*"):
                # Wildcard match
                prefix = trigger_lower[:-1]
                if check_lower.startswith(prefix):
                    score += 10
            elif check_lower == trigger_lower:
                score += 15
            elif trigger_lower in check_lower:
                score += 5
        
        # Match por keywords
        for keyword in pot.trigger_keywords:
            kw_lower = keyword.lower()
            if kw_lower in check_lower:
                score += 3
            if kw_lower in msg_lower:
                score += 2
        
        # Bonus por categoría REPAIR/INCIDENT
        if pot.category in (POTCategory.REPAIR, POTCategory.INCIDENT):
            score += 1
        
        if score > 0:
            scores.append((score, pot))
    
    if not scores:
        # Fallback: usar diagnostic_full si no hay match
        _log.info("No POT match for check_id=%s, using diagnostic_full", check_id)
        return get_pot("diagnostic_full")
    
    # Ordenar por score descendente
    scores.sort(key=lambda x: x[0], reverse=True)
    best_score, best_pot = scores[0]
    
    _log.info(
        "Selected POT %s (score=%d) for check_id=%s",
        best_pot.id, best_score, check_id
    )
    return best_pot


def get_pot_for_maintenance(maintenance_type: str = "daily") -> Optional[POT]:
    """
    Obtiene el POT de mantenimiento apropiado.
    
    Args:
        maintenance_type: "daily" o "weekly"
    
    Returns:
        POT de mantenimiento correspondiente
    """
    pot_id = f"maintenance_{maintenance_type}"
    return get_pot(pot_id)
