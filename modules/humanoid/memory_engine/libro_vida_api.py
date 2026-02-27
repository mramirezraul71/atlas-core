"""
API REST del Libro de Vida de ATLAS.

Endpoints:
  GET  /api/libro-vida/status          → estadísticas
  GET  /api/libro-vida/episodios       → listar recientes
  GET  /api/libro-vida/episodio/{id}   → detalle de un episodio
  POST /api/libro-vida/registrar       → registrar nuevo episodio
  POST /api/libro-vida/buscar          → buscar episodios similares
  GET  /api/libro-vida/reglas          → reglas aprendidas
  GET  /api/libro-vida/principios      → principios generales
  POST /api/libro-vida/principio       → registrar principio
  POST /api/libro-vida/planificar      → planificar tarea con LLM + Libro de Vida
  POST /api/libro-vida/registrar-resultado → registrar resultado de tarea ejecutada
"""
from __future__ import annotations

from fastapi import APIRouter
from pydantic import BaseModel, Field
from typing import Any, Dict, List, Optional
import time

router = APIRouter(prefix="/api/libro-vida", tags=["Libro de Vida"])


# ── Modelos Pydantic ─────────────────────────────────────────────────

class RegistrarEpisodioBody(BaseModel):
    tipo_tarea: str
    objetivo: str
    contexto_entorno: str = ""
    restricciones_seguridad: List[str] = []
    participantes: List[str] = []
    percepciones: Dict[str, Any] = {}
    acciones: List[Dict[str, Any]] = []
    resultado: Dict[str, Any] = {}
    feedback: Dict[str, Any] = {}
    lecciones: Dict[str, Any] = {}
    importancia: float = 0.5
    tags: List[str] = []
    valor_aprendizaje: str = "normal"


class BuscarBody(BaseModel):
    query: str = ""
    tipo_tarea: Optional[str] = None
    contexto: str = ""
    tags: List[str] = []
    limit: int = 5
    solo_exitosos: Optional[bool] = None


class PlanificarBody(BaseModel):
    tarea: str
    contexto: Dict[str, Any] = {}


class RegistrarResultadoBody(BaseModel):
    tarea: str
    tipo_tarea: str = "otro"
    plan_ejecutado: List[str] = []
    exito: bool = True
    errores: List[str] = []
    correcciones: List[str] = []
    lecciones: str = ""
    feedback_humano: str = ""
    contexto_entorno: str = ""
    metricas: Dict[str, float] = {}
    reglas_numericas: Dict[str, float] = {}


class PrincipioBody(BaseModel):
    categoria: str
    principio: str
    episodio_ids: List[str] = []
    confianza: float = 0.5


# ── Endpoints ────────────────────────────────────────────────────────

@router.get("/status")
def libro_vida_status():
    t0 = time.perf_counter()
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()
        stats = lv.get_stats()
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": True, **stats, "ms": ms}
    except Exception as e:
        return {"ok": False, "error": str(e), "ms": int((time.perf_counter() - t0) * 1000)}


@router.get("/episodios")
def libro_vida_listar(limit: int = 20):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()
        items = lv.listar_recientes(limit=limit)
        return {"ok": True, "data": items, "count": len(items), "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.get("/episodio/{ep_id}")
def libro_vida_detalle(ep_id: str):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()
        ep = lv.obtener_episodio(ep_id)
        if not ep:
            return {"ok": False, "error": "Episodio no encontrado"}
        return {"ok": True, "data": ep.to_dict(), "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/registrar")
def libro_vida_registrar(body: RegistrarEpisodioBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.memory_engine.libro_vida import (
            EpisodioVida, Percepcion, AccionEjecutada, Resultado,
            Feedback, Leccion, get_libro_vida,
        )

        percepciones = Percepcion(
            sensores=body.percepciones.get("sensores", {}),
            eventos=body.percepciones.get("eventos", []),
            anomalias=body.percepciones.get("anomalias", []),
        )
        acciones = [AccionEjecutada(**a) for a in body.acciones]
        resultado = Resultado(**body.resultado) if body.resultado else Resultado()
        feedback = Feedback(**body.feedback) if body.feedback else Feedback()
        lecciones = Leccion(**body.lecciones) if body.lecciones else Leccion()

        ep = EpisodioVida(
            tipo_tarea=body.tipo_tarea,
            objetivo=body.objetivo,
            contexto_entorno=body.contexto_entorno,
            restricciones_seguridad=body.restricciones_seguridad,
            participantes=body.participantes,
            percepciones=percepciones,
            acciones=acciones,
            resultado=resultado,
            feedback=feedback,
            lecciones=lecciones,
            importancia=body.importancia,
            tags=body.tags,
            valor_aprendizaje=body.valor_aprendizaje,
        )

        lv = get_libro_vida()
        ep_id = lv.registrar_episodio(ep)
        return {"ok": True, "id": ep_id, "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/buscar")
def libro_vida_buscar(body: BuscarBody):
    t0 = time.perf_counter()
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()

        if body.query and not body.tipo_tarea:
            eps = lv.buscar_por_texto(body.query, limit=body.limit)
        else:
            eps = lv.buscar_similares(
                tipo_tarea=body.tipo_tarea,
                objetivo=body.query,
                contexto=body.contexto,
                tags=body.tags,
                limit=body.limit,
                solo_exitosos=body.solo_exitosos,
            )

        data = [
            {
                "id": ep.id,
                "tipo_tarea": ep.tipo_tarea,
                "objetivo": ep.objetivo,
                "exito": ep.resultado.exito,
                "importancia": ep.importancia,
                "leccion": ep.lecciones.texto[:200],
                "timestamp": ep.timestamp,
            }
            for ep in eps
        ]
        return {"ok": True, "data": data, "count": len(data), "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.get("/reglas")
def libro_vida_reglas(tipo_tarea: Optional[str] = None, limit: int = 20):
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()
        reglas = lv.obtener_reglas(tipo_tarea=tipo_tarea, limit=limit)
        return {"ok": True, "data": reglas, "count": len(reglas)}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.get("/principios")
def libro_vida_principios(categoria: Optional[str] = None):
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()
        principios = lv.obtener_principios(categoria=categoria)
        return {"ok": True, "data": principios, "count": len(principios)}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.post("/principio")
def libro_vida_registrar_principio(body: PrincipioBody):
    try:
        from modules.humanoid.memory_engine.libro_vida import get_libro_vida
        lv = get_libro_vida()
        pid = lv.registrar_principio(
            categoria=body.categoria,
            principio=body.principio,
            episodio_ids=body.episodio_ids,
            confianza=body.confianza,
        )
        return {"ok": True, "id": pid}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/planificar")
def libro_vida_planificar(body: PlanificarBody):
    """Planifica una tarea usando el Libro de Vida + LLM."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.brain.libro_vida_planner import get_libro_vida_planner
        planner = get_libro_vida_planner()
        plan = planner.planificar(body.tarea, body.contexto or None)
        return {
            "ok": True,
            "objetivo": plan.objetivo,
            "episodios_consultados": plan.episodios_consultados,
            "episodios_resumen": plan.episodios_resumen,
            "plan_pasos": plan.plan_pasos,
            "explicacion_humano": plan.explicacion_humano,
            "respuesta_completa": plan.respuesta_completa,
            "modelo_usado": plan.modelo_usado,
            "ms": plan.ms,
        }
    except Exception as e:
        return {"ok": False, "error": str(e), "ms": int((time.perf_counter() - t0) * 1000)}


@router.post("/registrar-resultado")
def libro_vida_registrar_resultado(body: RegistrarResultadoBody):
    """Registra el resultado de una tarea ejecutada en el Libro de Vida."""
    t0 = time.perf_counter()
    try:
        from modules.humanoid.brain.libro_vida_planner import get_libro_vida_planner
        planner = get_libro_vida_planner()
        ep_id = planner.registrar_resultado(
            tarea=body.tarea,
            tipo_tarea=body.tipo_tarea,
            plan_ejecutado=body.plan_ejecutado,
            exito=body.exito,
            errores=body.errores,
            correcciones=body.correcciones,
            lecciones=body.lecciones,
            feedback_humano=body.feedback_humano,
            contexto_entorno=body.contexto_entorno,
            metricas=body.metricas,
            reglas_numericas=body.reglas_numericas,
        )
        return {"ok": True, "episodio_id": ep_id, "ms": int((time.perf_counter() - t0) * 1000)}
    except Exception as e:
        return {"ok": False, "error": str(e)}
