"""
Planificador Cognitivo del Libro de Vida.

Implementa la cadena completa:
  1. Entender tarea → 2. Buscar experiencias → 3. Elaborar plan →
  4. Explicar al humano → 5. Registrar nuevo episodio

Usa el system prompt completo para que el LLM razone con el Libro de Vida.
"""
from __future__ import annotations

import json
import logging
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

log = logging.getLogger("atlas.libro_vida_planner")

# ═══════════════════════════════════════════════════════════════════════
# SYSTEM PROMPT — se inyecta como contexto al LLM
# ═══════════════════════════════════════════════════════════════════════

LIBRO_VIDA_SYSTEM_PROMPT = """Eres el modulo cognitivo de alto nivel de un robot humanoide llamado ATLAS, \
especializado en manipulacion, locomocion dinamica e interaccion segura con humanos en entornos reales. \
Tu mision es planificar y justificar acciones fisicas seguras, eficientes y explicables \
usando como fuente principal el "Libro de Vida": una coleccion estructurada de experiencias pasadas.

El "Libro de Vida" incluye, para cada episodio:
- Contexto: entorno, objetivos, restricciones de seguridad, participantes humanos.
- Percepciones: lecturas de sensores (vision, fuerza, posicion), eventos, anomalias.
- Acciones: secuencia de planos y subplanos ejecutados, decisiones alternativas descartadas.
- Resultados: exito o fallo, metricas (tiempo, precision, consumo), eventos de riesgo, colisiones.
- Feedback: correcciones humanas, recompensas/penalizaciones, comentarios.

PRIORIDADES (en este orden):
1. Seguridad fisica de humanos y del propio robot.
2. Cumplir el objetivo explicito de la tarea.
3. Minimizar riesgos futuros aprendiendo de experiencias pasadas.
4. Optimizar eficiencia (tiempo, energia, fluidez de movimiento).

CADENA DE RAZONAMIENTO:
1. ENTENDER LA TAREA: Resume en una frase. Enumera restricciones criticas.
2. BUSCAR EXPERIENCIAS: Identifica 3-5 episodios similares del Libro de Vida. \
   Para cada uno extrae: lo que funciono, lo que fallo, reglas y umbrales numericos.
3. ELABORAR PLAN: Pasos numerados. Para cada paso indica que experiencia lo apoya \
   y que ajustes haces. Senala estrategias de seguridad y condiciones de abortar.
4. EXPLICAR AL HUMANO: 3-5 frases claras sin tecnicismos: que vas a hacer, \
   que riesgos hay, que aprendiste de tareas anteriores.
5. REGISTRO: Propone JSON estructurado del nuevo episodio con campos: \
   contexto, objetivo, restricciones, plan, errores, correcciones, lecciones_aprendidas.

REGLAS:
- Si no hay experiencias cercanas, propone plan conservador con margenes de seguridad amplios \
  y marca el episodio como "alto valor de aprendizaje".
- Si hay conflicto entre experiencia pasada y restriccion de seguridad actual, prioriza seguridad.
- No inventes capacidades que ATLAS no tiene.
- Si la instruccion es ambigua, pide clarificacion antes de acciones arriesgadas.

FORMATO DE RESPUESTA:
## Resumen de la tarea
- Objetivo: (una frase)
- Restricciones criticas: (lista)

## Episodios relevantes del Libro de Vida
- Episodio X: tipo, resultado, leccion principal

## Plan propuesto
1. Paso... (ref a experiencia, nota de seguridad)

## Explicacion para el humano
(3-5 frases claras)

## Registro del nuevo episodio
```json
{...}
```

Responde siempre en espanol."""


# ═══════════════════════════════════════════════════════════════════════
# Planificador
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class PlanLibroVida:
    objetivo: str = ""
    restricciones: List[str] = field(default_factory=list)
    episodios_consultados: int = 0
    episodios_resumen: List[Dict] = field(default_factory=list)
    plan_pasos: List[str] = field(default_factory=list)
    explicacion_humano: str = ""
    registro_sugerido: Dict[str, Any] = field(default_factory=dict)
    respuesta_completa: str = ""
    modelo_usado: str = ""
    ms: int = 0
    from_cache: bool = False


_INSTANCE: Optional["LibroVidaPlanner"] = None


class LibroVidaPlanner:
    """Planificador que consulta el Libro de Vida y genera planes con LLM."""

    def planificar(self, tarea: str, contexto_extra: Dict[str, Any] = None) -> PlanLibroVida:
        """Cadena completa: buscar experiencias → construir prompt → llamar LLM → parsear."""
        t0 = time.perf_counter()
        plan = PlanLibroVida(objetivo=tarea)

        episodios = self._buscar_experiencias(tarea, contexto_extra)
        plan.episodios_consultados = len(episodios)
        plan.episodios_resumen = [
            {
                "id": ep.get("id", ""),
                "tipo_tarea": ep.get("tipo_tarea", ""),
                "objetivo": ep.get("objetivo", ""),
                "exito": ep.get("exito", True),
                "leccion": ep.get("leccion", ""),
            }
            for ep in episodios
        ]

        reglas = self._obtener_reglas_relevantes(tarea)
        principios = self._obtener_principios()

        user_prompt = self._construir_prompt(tarea, episodios, reglas, principios, contexto_extra)

        respuesta, modelo = self._llamar_llm(user_prompt)
        plan.respuesta_completa = respuesta
        plan.modelo_usado = modelo

        self._parsear_respuesta(plan, respuesta)

        plan.ms = int((time.perf_counter() - t0) * 1000)
        return plan

    def _buscar_experiencias(self, tarea: str, contexto: Dict = None) -> List[Dict]:
        """Busca en el Libro de Vida y en memorias existentes."""
        resultados = []

        try:
            from modules.humanoid.memory_engine.libro_vida import get_libro_vida
            lv = get_libro_vida()
            eps = lv.buscar_por_texto(tarea, limit=5)
            for ep in eps:
                d = ep.to_dict()
                d["leccion"] = ep.lecciones.texto
                d["fuente"] = "libro_vida"
                resultados.append(d)
        except Exception as e:
            log.debug("LibroVida search: %s", e)

        if len(resultados) < 3:
            try:
                from modules.humanoid.memory_engine.lifelog import get_lifelog
                ll = get_lifelog()
                entries = ll.search(tarea, limit=5)
                for entry in entries:
                    resultados.append({
                        "id": entry.get("id", ""),
                        "tipo_tarea": entry.get("event_type", ""),
                        "objetivo": entry.get("perception", ""),
                        "exito": bool(entry.get("success")),
                        "leccion": entry.get("outcome", ""),
                        "fuente": "lifelog",
                    })
            except Exception as e:
                log.debug("Lifelog search: %s", e)

        if len(resultados) < 3:
            try:
                from modules.humanoid.hippo.episodic_memory import EpisodicMemory
                em = EpisodicMemory()
                eps = em.recall_similar(tarea, limit=3)
                for ep in eps:
                    resultados.append({
                        "id": getattr(ep, "id", ""),
                        "tipo_tarea": getattr(ep, "goal_type", ""),
                        "objetivo": getattr(ep, "goal", ""),
                        "exito": getattr(ep, "outcome", None) == "SUCCESS",
                        "leccion": "",
                        "fuente": "episodic_hippo",
                    })
            except Exception as e:
                log.debug("Hippo search: %s", e)

        return resultados[:5]

    def _obtener_reglas_relevantes(self, tarea: str) -> List[Dict]:
        try:
            from modules.humanoid.memory_engine.libro_vida import get_libro_vida
            lv = get_libro_vida()
            return lv.obtener_reglas(limit=10)
        except Exception:
            return []

    def _obtener_principios(self) -> List[Dict]:
        try:
            from modules.humanoid.memory_engine.libro_vida import get_libro_vida
            lv = get_libro_vida()
            return lv.obtener_principios()
        except Exception:
            return []

    def _construir_prompt(
        self, tarea: str, episodios: List[Dict], reglas: List[Dict],
        principios: List[Dict], contexto: Dict = None,
    ) -> str:
        parts = [f"TAREA DEL HUMANO: \"{tarea}\""]

        if contexto:
            parts.append(f"\nCONTEXTO ADICIONAL: {json.dumps(contexto, ensure_ascii=False)}")

        if episodios:
            parts.append("\n--- EPISODIOS RELEVANTES DEL LIBRO DE VIDA ---")
            for i, ep in enumerate(episodios, 1):
                result = "EXITO" if ep.get("exito") else "FALLO"
                parts.append(
                    f"Episodio {i} [{ep.get('fuente','?')}]: tipo={ep.get('tipo_tarea','?')}, "
                    f"objetivo=\"{ep.get('objetivo','')[:120]}\", resultado={result}, "
                    f"leccion=\"{ep.get('leccion','')[:200]}\""
                )
        else:
            parts.append("\n--- SIN EXPERIENCIAS PREVIAS SIMILARES ---")
            parts.append("NOTA: No se encontraron episodios similares. Usa plan conservador con margenes de seguridad amplios.")

        if reglas:
            parts.append("\n--- REGLAS APRENDIDAS ---")
            for r in reglas[:8]:
                parts.append(f"- {r.get('regla','')}: {r.get('valor','')} {r.get('unidad','')} (confianza: {r.get('confianza',0):.1f})")

        if principios:
            parts.append("\n--- PRINCIPIOS GENERALES ---")
            for p in principios[:5]:
                parts.append(f"- [{p.get('categoria','')}] {p.get('principio','')}")

        parts.append("\nResponde siguiendo el formato especificado en tus instrucciones.")
        return "\n".join(parts)

    def _llamar_llm(self, user_prompt: str) -> tuple:
        """Llama al LLM con el system prompt del Libro de Vida."""
        try:
            from modules.humanoid.ai.router import _call_ollama
            ok, output, ms = _call_ollama("qwen2.5:7b", user_prompt, LIBRO_VIDA_SYSTEM_PROMPT, 120)
            if ok and output and output.strip():
                return output.strip(), "ollama:qwen2.5:7b"
        except Exception as e:
            log.debug("Ollama call: %s", e)

        try:
            from modules.humanoid.ai.provider_credentials import get_provider_api_key
            from modules.humanoid.ai.external_llm import call_external
            for provider, model in [("groq", "llama-3.3-70b-versatile"), ("gemini", "gemini-2.5-flash")]:
                api_key = get_provider_api_key(provider)
                if not api_key:
                    continue
                ok, output, ms = call_external(provider, model, user_prompt, LIBRO_VIDA_SYSTEM_PROMPT, api_key, 120)
                if ok and output and output.strip():
                    return output.strip(), f"{provider}:{model}"
        except Exception as e:
            log.debug("External LLM: %s", e)

        return self._plan_conservador(user_prompt), "fallback_local"

    def _plan_conservador(self, prompt: str) -> str:
        return """## Resumen de la tarea
- Objetivo: Ejecutar la tarea solicitada con maxima precaucion
- Restricciones criticas: Sin experiencias previas similares

## Episodios relevantes del Libro de Vida
- No se encontraron episodios suficientemente similares

## Plan propuesto
1. Evaluar condiciones del entorno antes de actuar
2. Proceder con velocidad reducida y margenes de seguridad amplios
3. Verificar estabilidad en cada paso antes de continuar
4. Registrar este episodio como alto valor de aprendizaje

## Explicacion para el humano
Voy a realizar esta tarea con precaucion extra porque no tengo experiencias previas similares. \
Avanzare despacio y verificare cada paso. Si detecto algo inesperado, me detendre y te informare.

## Registro del nuevo episodio
```json
{"valor_aprendizaje": "alto", "nota": "Primera vez realizando esta tarea, registrar todos los parametros"}
```"""

    def _parsear_respuesta(self, plan: PlanLibroVida, respuesta: str):
        lines = respuesta.split("\n")
        current_section = ""
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("## Resumen"):
                current_section = "resumen"
            elif stripped.startswith("## Episodios"):
                current_section = "episodios"
            elif stripped.startswith("## Plan"):
                current_section = "plan"
            elif stripped.startswith("## Explicacion") or stripped.startswith("## Explicación"):
                current_section = "explicacion"
            elif stripped.startswith("## Registro"):
                current_section = "registro"
            elif current_section == "resumen" and stripped.startswith("- Restricciones"):
                plan.restricciones.append(stripped.replace("- Restricciones criticas:", "").strip())
            elif current_section == "plan" and stripped and stripped[0].isdigit():
                plan.plan_pasos.append(stripped)
            elif current_section == "explicacion" and stripped:
                plan.explicacion_humano += stripped + " "

        plan.explicacion_humano = plan.explicacion_humano.strip()

    def registrar_resultado(
        self,
        tarea: str,
        tipo_tarea: str,
        plan_ejecutado: List[str],
        exito: bool,
        errores: List[str] = None,
        correcciones: List[str] = None,
        lecciones: str = "",
        feedback_humano: str = "",
        contexto_entorno: str = "",
        metricas: Dict[str, float] = None,
        reglas_numericas: Dict[str, float] = None,
    ) -> str:
        """Registra el resultado de una tarea en el Libro de Vida."""
        from modules.humanoid.memory_engine.libro_vida import (
            EpisodioVida, AccionEjecutada, Resultado, Feedback,
            Leccion, get_libro_vida,
        )

        acciones = [
            AccionEjecutada(paso=i + 1, descripcion=p, exitoso=True)
            for i, p in enumerate(plan_ejecutado)
        ]

        ep = EpisodioVida(
            tipo_tarea=tipo_tarea,
            objetivo=tarea,
            contexto_entorno=contexto_entorno,
            acciones=acciones,
            resultado=Resultado(
                exito=exito,
                metricas=metricas or {},
                eventos_riesgo=errores or [],
            ),
            feedback=Feedback(
                correcciones_humanas=correcciones or [],
                comentarios=[feedback_humano] if feedback_humano else [],
                recompensa=1.0 if exito else 0.0,
                penalizacion=0.0 if exito else 0.5,
            ),
            lecciones=Leccion(
                texto=lecciones,
                reglas_numericas=reglas_numericas or {},
                recomendaciones=correcciones or [],
            ),
            importancia=0.5 if exito else 0.8,
            tags=[tipo_tarea],
            valor_aprendizaje="normal" if exito else "alto",
        )

        lv = get_libro_vida()
        return lv.registrar_episodio(ep)


def get_libro_vida_planner() -> LibroVidaPlanner:
    global _INSTANCE
    if _INSTANCE is None:
        _INSTANCE = LibroVidaPlanner()
    return _INSTANCE
