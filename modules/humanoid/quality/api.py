"""
Quality API: Endpoints para gestión de POTs.
"""
from __future__ import annotations

from fastapi import APIRouter
from pydantic import BaseModel
from typing import Optional, List
import json

router = APIRouter(prefix="/quality", tags=["Quality"])


class ExecutePOTBody(BaseModel):
    pot_id: str
    context: Optional[dict] = None
    dry_run: Optional[bool] = False


@router.get("/pots")
def list_pots(
    category: Optional[str] = None,
    severity: Optional[str] = None,
    tags: Optional[str] = None,
):
    """
    Lista todos los POTs disponibles.
    
    Query params:
    - category: Filtrar por categoría (repair, maintenance, incident, diagnostic, etc)
    - severity: Filtrar por severidad (low, medium, high, critical)
    - tags: Filtrar por tags (comma-separated)
    """
    try:
        from modules.humanoid.quality import list_pots as _list_pots
        tag_list = [t.strip() for t in tags.split(",")] if tags else None
        pots = _list_pots(category=category, severity=severity, tags=tag_list)
        return {"ok": True, "pots": pots, "count": len(pots)}
    except Exception as e:
        return {"ok": False, "pots": [], "error": str(e)}


@router.get("/pots/{pot_id}")
def get_pot_details(pot_id: str):
    """
    Obtiene detalles completos de un POT específico.
    
    Incluye:
    - Todos los pasos con descripciones tutoriales
    - Requisitos previos
    - Best practices y warnings
    - Triggers y keywords
    """
    try:
        from modules.humanoid.quality import get_pot
        pot = get_pot(pot_id)
        if not pot:
            return {"ok": False, "error": f"POT not found: {pot_id}"}
        return {"ok": True, "pot": pot.to_dict()}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/pots/execute")
def execute_pot(body: ExecutePOTBody):
    """
    Ejecuta un POT manualmente.
    
    Body:
    - pot_id: ID del POT a ejecutar
    - context: Contexto inicial (opcional)
    - dry_run: Si true, solo simula (opcional)
    """
    try:
        from modules.humanoid.quality import get_pot, execute_pot as _execute_pot
        
        pot = get_pot(body.pot_id)
        if not pot:
            return {"ok": False, "error": f"POT not found: {body.pot_id}"}
        
        result = _execute_pot(
            pot,
            context=body.context or {},
            dry_run=body.dry_run or False,
        )
        return {
            "ok": result.ok,
            "pot_id": result.pot_id,
            "elapsed_ms": result.elapsed_ms,
            "steps_ok": result.steps_ok,
            "steps_failed": result.steps_failed,
            "steps_skipped": result.steps_skipped,
            "report_path": result.report_path,
            "rollback_executed": result.rollback_executed,
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/pots/match")
def match_pot_for_incident(check_id: str, message: Optional[str] = None):
    """
    Encuentra el POT más apropiado para un incidente dado.
    
    Query params:
    - check_id: ID del check que generó el incidente
    - message: Mensaje del incidente (opcional)
    """
    try:
        from modules.humanoid.quality import get_pot_by_incident
        pot = get_pot_by_incident(check_id=check_id, message=message or "")
        if not pot:
            return {"ok": True, "match": None, "message": "No matching POT found"}
        return {
            "ok": True,
            "match": {
                "pot_id": pot.id,
                "pot_name": pot.name,
                "category": pot.category.value if hasattr(pot.category, 'value') else pot.category,
                "severity": pot.severity.value if hasattr(pot.severity, 'value') else pot.severity,
            }
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/reports")
def list_reports(limit: int = 20):
    """Lista los últimos reportes de ejecución de POTs."""
    try:
        from pathlib import Path
        reports_dir = Path(__file__).parent / "reports"
        if not reports_dir.exists():
            return {"ok": True, "reports": [], "count": 0}
        
        files = sorted(reports_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
        reports = []
        for f in files:
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                reports.append({
                    "filename": f.name,
                    "pot_id": data.get("pot_id"),
                    "pot_name": data.get("pot_name"),
                    "ok": data.get("ok"),
                    "started_at": data.get("started_at"),
                    "elapsed_ms": data.get("elapsed_ms"),
                    "steps_ok": data.get("steps_ok"),
                    "steps_failed": data.get("steps_failed"),
                })
            except Exception:
                reports.append({"filename": f.name, "error": "parse_failed"})
        return {"ok": True, "reports": reports, "count": len(reports)}
    except Exception as e:
        return {"ok": False, "reports": [], "error": str(e)}


@router.get("/reports/{filename}")
def get_report(filename: str):
    """Obtiene un reporte específico de ejecución de POT."""
    try:
        from pathlib import Path
        reports_dir = Path(__file__).parent / "reports"
        report_path = reports_dir / filename
        
        if not report_path.exists():
            return {"ok": False, "error": f"Report not found: {filename}"}
        
        data = json.loads(report_path.read_text(encoding="utf-8"))
        return {"ok": True, "report": data}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/categories")
def list_categories():
    """Lista todas las categorías de POTs disponibles."""
    try:
        from modules.humanoid.quality.models import POTCategory
        categories = [
            {
                "id": c.value,
                "name": c.name,
                "description": {
                    "repair": "Reparaciones correctivas",
                    "maintenance": "Mantenimiento preventivo",
                    "incident": "Gestión de incidentes",
                    "diagnostic": "Diagnósticos",
                    "deployment": "Despliegues",
                    "recovery": "Recuperación de desastres",
                    "security": "Procedimientos de seguridad",
                    "calibration": "Calibración de sensores/cámaras",
                    "upgrade": "Actualizaciones de sistema",
                }.get(c.value, "")
            }
            for c in POTCategory
        ]
        return {"ok": True, "categories": categories}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/tutorial/{pot_id}")
def get_pot_tutorial(pot_id: str):
    """
    Obtiene el tutorial completo de un POT.
    
    Incluye:
    - Overview tutorial
    - Best practices
    - Warnings
    - Notas de cada paso
    - Troubleshooting
    """
    try:
        from modules.humanoid.quality import get_pot
        pot = get_pot(pot_id)
        if not pot:
            return {"ok": False, "error": f"POT not found: {pot_id}"}
        
        steps_tutorial = []
        for step in pot.steps:
            steps_tutorial.append({
                "id": step.id,
                "name": step.name,
                "description": step.description,
                "tutorial_notes": step.tutorial_notes,
                "common_errors": step.common_errors,
                "troubleshooting": step.troubleshooting,
            })
        
        return {
            "ok": True,
            "pot_id": pot.id,
            "pot_name": pot.name,
            "overview": pot.tutorial_overview,
            "best_practices": pot.best_practices,
            "warnings": pot.warnings,
            "prerequisites": pot.prerequisites,
            "objectives": pot.objectives,
            "success_criteria": pot.success_criteria,
            "estimated_minutes": pot.estimated_duration_minutes,
            "steps": steps_tutorial,
            "related_pots": pot.related_pots,
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}
