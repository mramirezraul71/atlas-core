"""
API REST para el sistema de Tutorías y Visitas
Integración con FastAPI
"""

from datetime import datetime
from typing import List, Optional, Dict, Any
from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

from .models import (
    TipoVisita, NivelEvaluacion, EstadoRecomendacion, 
    PrioridadRecomendacion
)
from .manager import TutoriasManager
from .reports import ReportGenerator


# ==================== MODELOS PYDANTIC ====================

class EspecialistaCreate(BaseModel):
    nombre: str
    rol: str
    especialidad: str
    email: str = ""


class EvaluacionCreate(BaseModel):
    aspecto: str
    nivel: str = "ACEPTABLE"  # EXCELENTE, BUENO, ACEPTABLE, MEJORABLE, CRITICO
    puntuacion: float = 3.0
    comentario: str = ""


class RecomendacionCreate(BaseModel):
    titulo: str
    descripcion: str
    modulo_afectado: str
    prioridad: str = "MEDIA"  # CRITICA, ALTA, MEDIA, BAJA, OPCIONAL
    pasos_implementacion: List[str] = []


class InformeCreate(BaseModel):
    titulo: str
    resumen: str
    contenido: str
    objetivos_cumplidos: List[str] = []
    objetivos_pendientes: List[str] = []
    evaluaciones: List[EvaluacionCreate] = []
    recomendaciones: List[RecomendacionCreate] = []
    proximos_pasos: List[str] = []


class VisitaCreate(BaseModel):
    tipo: str = "tutoria"  # tutoria, revision, auditoria, capacitacion, etc.
    especialista_id: str
    motivo: str
    modulos_revisados: List[str] = []
    objetivos: List[str] = []


class SeguimientoUpdate(BaseModel):
    hito_idx: int
    completado: bool = True


# ==================== ROUTER ====================

router = APIRouter(prefix="/tutorias", tags=["Tutorías y Visitas"])

# Instancias globales
_manager: Optional[TutoriasManager] = None
_reporter: Optional[ReportGenerator] = None


def get_manager() -> TutoriasManager:
    global _manager
    if _manager is None:
        _manager = TutoriasManager()
    return _manager


def get_reporter() -> ReportGenerator:
    global _reporter
    if _reporter is None:
        _reporter = ReportGenerator()
    return _reporter


# ==================== ESPECIALISTAS ====================

@router.post("/especialistas", summary="Registrar nuevo especialista")
def crear_especialista(data: EspecialistaCreate):
    """Registra un nuevo especialista en el sistema"""
    from .models import Especialista
    
    esp = Especialista(
        nombre=data.nombre,
        rol=data.rol,
        especialidad=data.especialidad,
        email=data.email
    )
    esp.generar_firma()
    
    esp_id = get_manager().registrar_especialista(esp)
    
    return {
        "ok": True,
        "message": f"Especialista {data.nombre} registrado",
        "especialista_id": esp_id,
        "firma_digital": esp.firma_digital
    }


@router.get("/especialistas", summary="Listar especialistas")
def listar_especialistas():
    """Lista todos los especialistas registrados"""
    especialistas = get_manager().listar_especialistas()
    return {
        "ok": True,
        "total": len(especialistas),
        "especialistas": [e.to_dict() for e in especialistas]
    }


@router.get("/especialistas/{especialista_id}", summary="Obtener especialista")
def obtener_especialista(especialista_id: str):
    """Obtiene un especialista por ID"""
    esp = get_manager().obtener_especialista(especialista_id)
    if not esp:
        raise HTTPException(status_code=404, detail="Especialista no encontrado")
    return {"ok": True, "especialista": esp.to_dict()}


# ==================== VISITAS ====================

@router.post("/visitas", summary="Iniciar nueva visita")
def iniciar_visita(data: VisitaCreate):
    """Inicia una nueva visita de especialista"""
    manager = get_manager()
    
    # Obtener especialista
    esp = manager.obtener_especialista(data.especialista_id)
    if not esp:
        raise HTTPException(status_code=404, detail="Especialista no encontrado")
    
    # Crear visita
    try:
        tipo = TipoVisita(data.tipo)
    except ValueError:
        tipo = TipoVisita.TUTORIA
    
    visita = manager.iniciar_visita(
        especialista=esp,
        tipo=tipo,
        motivo=data.motivo,
        modulos=data.modulos_revisados,
        objetivos=data.objetivos
    )
    
    return {
        "ok": True,
        "message": f"Visita {tipo.value} iniciada",
        "visita_id": visita.id,
        "especialista": esp.nombre,
        "fecha_inicio": visita.fecha_inicio.isoformat()
    }


@router.post("/visitas/{visita_id}/finalizar", summary="Finalizar visita con informe")
def finalizar_visita(visita_id: str, informe_data: InformeCreate):
    """Finaliza una visita y registra el informe firmado"""
    manager = get_manager()
    
    from .models import Informe, Evaluacion, Recomendacion, NivelEvaluacion, PrioridadRecomendacion
    
    # Crear evaluaciones
    evaluaciones = []
    for ev in informe_data.evaluaciones:
        try:
            nivel = NivelEvaluacion[ev.nivel]
        except KeyError:
            nivel = NivelEvaluacion.ACEPTABLE
        
        evaluaciones.append(Evaluacion(
            aspecto=ev.aspecto,
            nivel=nivel,
            puntuacion=ev.puntuacion,
            comentario=ev.comentario
        ))
    
    # Crear recomendaciones
    recomendaciones = []
    for rec in informe_data.recomendaciones:
        try:
            prioridad = PrioridadRecomendacion[rec.prioridad]
        except KeyError:
            prioridad = PrioridadRecomendacion.MEDIA
        
        recomendaciones.append(Recomendacion(
            titulo=rec.titulo,
            descripcion=rec.descripcion,
            modulo_afectado=rec.modulo_afectado,
            prioridad=prioridad,
            pasos_implementacion=rec.pasos_implementacion
        ))
    
    # Crear informe
    informe = Informe(
        titulo=informe_data.titulo,
        resumen=informe_data.resumen,
        contenido=informe_data.contenido,
        objetivos_cumplidos=informe_data.objetivos_cumplidos,
        objetivos_pendientes=informe_data.objetivos_pendientes,
        evaluaciones=evaluaciones,
        recomendaciones=recomendaciones,
        proximos_pasos=informe_data.proximos_pasos
    )
    
    # Finalizar visita
    try:
        visita = manager.finalizar_visita(informe)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    
    # Generar reporte
    reporter = get_reporter()
    report_path = reporter.generar_informe_visita(visita, "html")
    
    return {
        "ok": True,
        "message": "Visita finalizada e informe firmado",
        "visita_id": visita.id,
        "duracion_minutos": visita.duracion_minutos,
        "informe": {
            "id": informe.id,
            "firmado": informe.firmado,
            "firma": informe.firma_especialista,
            "hash": informe.hash_verificacion,
            "puntuacion_global": informe.calcular_puntuacion_global()
        },
        "recomendaciones_creadas": len(recomendaciones),
        "reporte_generado": report_path
    }


@router.get("/visitas", summary="Listar visitas")
def listar_visitas(
    limite: int = Query(50, ge=1, le=200),
    tipo: str = None,
    especialista_id: str = None,
    solo_completadas: bool = False
):
    """Lista visitas con filtros opcionales"""
    manager = get_manager()
    
    tipo_enum = None
    if tipo:
        try:
            tipo_enum = TipoVisita(tipo)
        except ValueError:
            pass
    
    visitas = manager.listar_visitas(
        limite=limite,
        tipo=tipo_enum,
        especialista_id=especialista_id,
        solo_completadas=solo_completadas
    )
    
    return {
        "ok": True,
        "total": len(visitas),
        "visitas": [v.to_dict() for v in visitas]
    }


@router.get("/visitas/{visita_id}", summary="Obtener visita")
def obtener_visita(visita_id: str):
    """Obtiene una visita por ID"""
    visita = get_manager().obtener_visita(visita_id)
    if not visita:
        raise HTTPException(status_code=404, detail="Visita no encontrada")
    return {"ok": True, "visita": visita.to_dict()}


# ==================== INFORMES ====================

@router.get("/informes", summary="Listar informes")
def listar_informes(
    limite: int = Query(50, ge=1, le=200),
    solo_firmados: bool = False
):
    """Lista informes"""
    informes = get_manager().listar_informes(limite, solo_firmados)
    return {
        "ok": True,
        "total": len(informes),
        "informes": [i.to_dict() for i in informes]
    }


@router.get("/informes/{informe_id}", summary="Obtener informe")
def obtener_informe(informe_id: str):
    """Obtiene un informe por ID"""
    informe = get_manager().obtener_informe(informe_id)
    if not informe:
        raise HTTPException(status_code=404, detail="Informe no encontrado")
    return {"ok": True, "informe": informe.to_dict()}


@router.get("/informes/{informe_id}/reporte", summary="Generar reporte de informe")
def generar_reporte_informe(
    informe_id: str,
    formato: str = Query("html", regex="^(html|markdown|json|txt)$")
):
    """Genera un reporte del informe en el formato especificado"""
    manager = get_manager()
    informe = manager.obtener_informe(informe_id)
    if not informe:
        raise HTTPException(status_code=404, detail="Informe no encontrado")
    
    # Buscar visita asociada
    # Por simplicidad, crear una visita temporal para el reporte
    from .models import Visita
    visita = Visita(informe=informe)
    
    reporter = get_reporter()
    path = reporter.generar_informe_visita(visita, formato)
    
    return {
        "ok": True,
        "formato": formato,
        "path": path
    }


# ==================== RECOMENDACIONES ====================

@router.get("/recomendaciones", summary="Listar recomendaciones")
def listar_recomendaciones(
    estado: str = None,
    modulo: str = None,
    limite: int = Query(50, ge=1, le=200)
):
    """Lista recomendaciones con filtros"""
    manager = get_manager()
    
    estado_enum = None
    if estado:
        try:
            estado_enum = EstadoRecomendacion[estado]
        except KeyError:
            pass
    
    recs = manager.listar_recomendaciones(
        estado=estado_enum,
        modulo=modulo,
        limite=limite
    )
    
    return {
        "ok": True,
        "total": len(recs),
        "recomendaciones": [r.to_dict() for r in recs]
    }


@router.put("/recomendaciones/{recomendacion_id}/estado", summary="Actualizar estado de recomendación")
def actualizar_estado_recomendacion(
    recomendacion_id: str,
    estado: str,
    nota: str = None
):
    """Actualiza el estado de una recomendación"""
    try:
        estado_enum = EstadoRecomendacion[estado]
    except KeyError:
        raise HTTPException(status_code=400, detail=f"Estado inválido: {estado}")
    
    get_manager().actualizar_estado_recomendacion(
        recomendacion_id,
        estado_enum,
        nota
    )
    
    return {
        "ok": True,
        "message": f"Estado actualizado a {estado}",
        "recomendacion_id": recomendacion_id
    }


# ==================== SEGUIMIENTOS ====================

@router.post("/seguimientos", summary="Crear seguimiento de recomendación")
def crear_seguimiento(recomendacion_id: str, responsable: str = "ATLAS"):
    """Crea un seguimiento para una recomendación"""
    manager = get_manager()
    
    # Obtener recomendación (simplificado)
    recs = manager.listar_recomendaciones()
    rec = next((r for r in recs if r.id == recomendacion_id), None)
    
    if not rec:
        raise HTTPException(status_code=404, detail="Recomendación no encontrada")
    
    seguimiento = manager.crear_seguimiento(rec, responsable)
    
    return {
        "ok": True,
        "message": "Seguimiento creado",
        "seguimiento_id": seguimiento.id,
        "hitos": len(seguimiento.hitos)
    }


@router.get("/seguimientos", summary="Listar seguimientos")
def listar_seguimientos(
    solo_activos: bool = True,
    limite: int = Query(50, ge=1, le=200)
):
    """Lista seguimientos de mejoras"""
    seguimientos = get_manager().listar_seguimientos(solo_activos, limite)
    return {
        "ok": True,
        "total": len(seguimientos),
        "seguimientos": [s.to_dict() for s in seguimientos]
    }


@router.put("/seguimientos/{seguimiento_id}/hito", summary="Actualizar hito de seguimiento")
def actualizar_hito_seguimiento(seguimiento_id: str, data: SeguimientoUpdate):
    """Marca un hito como completado en un seguimiento"""
    get_manager().actualizar_seguimiento(
        seguimiento_id,
        data.hito_idx,
        data.completado
    )
    
    return {
        "ok": True,
        "message": f"Hito {data.hito_idx} actualizado",
        "seguimiento_id": seguimiento_id
    }


# ==================== ESTADÍSTICAS ====================

@router.get("/estadisticas", summary="Obtener estadísticas del sistema")
def obtener_estadisticas():
    """Obtiene estadísticas generales del sistema de tutorías"""
    stats = get_manager().obtener_estadisticas()
    return {"ok": True, "estadisticas": stats}


@router.get("/dashboard", summary="Generar dashboard HTML")
def generar_dashboard():
    """Genera un dashboard HTML con el estado actual"""
    manager = get_manager()
    reporter = get_reporter()
    
    stats = manager.obtener_estadisticas()
    seguimientos = manager.listar_seguimientos(solo_activos=True)
    
    path = reporter.generar_dashboard_estado(stats, seguimientos)
    
    return {
        "ok": True,
        "dashboard_path": path
    }
