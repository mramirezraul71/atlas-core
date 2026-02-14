# Aprendizaje progresivo ATLAS

Arquitectura del sistema de aprendizaje continuo del robot: **Percibe → Razonar → Actuar → Aprender → Consolidar**.

## Componentes

| Módulo | Ruta | Descripción |
|--------|------|-------------|
| Base de conocimiento inicial | `brain/knowledge/initial_knowledge.py` | Conceptos, skills, reglas y relaciones causales con las que el robot “nace”. Persistencia en `brain/knowledge/initial_kb.json`. |
| Detector de incertidumbre | `brain/learning/uncertainty_detector.py` | Decide cuándo el robot no sabe: confidence bajo, pocas experiencias similares, desacuerdo ensemble, fallos repetidos. |
| Consultor IA | `brain/learning/ai_consultant.py` | Consulta al LLM de ATLAS (router free-first) cuando hay incertidumbre; extrae reasoning, acción y conocimiento en formato estructurado. |
| Consolidador | `brain/learning/knowledge_consolidator.py` | Consolida experiencias en patrones, generalizaciones, reglas causales y contradicciones; usa episódica o semántica; disparo periódico (p. ej. cada hora). |
| Memoria episódica | `brain/learning/episodic_memory.py` | SQLite en `logs/learning_episodic.sqlite`: episodios (situation_type, action, outcome, success). Usada por el loop y el consolidador. |
| Loop continuo | `brain/learning/continual_learning_loop.py` | Orquesta: entender → decidir (con incertidumbre) → ejecutar (hook opcional `action_executor`) → aprender (semántica + episódica) → consolidar. |

## Memoria

- **Semántica (FAISS + SQLite):** `modules/humanoid/memory_engine/semantic_memory.py` — experiencias con embeddings, `recall_similar`, `add_experience`. Si no está disponible (p. ej. sin sentence_transformers), se usa un stub (sin fallo de arranque).
- **Episódica (SQLite):** `brain/learning/episodic_memory.py` — tabla `episodes` con tipo de situación, acción, resultado y éxito/fallo. El loop escribe cada experiencia; el consolidador lee episodios recientes para patrones, generalizaciones y reglas causales. Opcional: si no se puede importar, el loop y el consolidador siguen funcionando solo con semántica.

## API (atlas_http_api)

- **POST /api/learning/process-situation** — Body: `description`, `type`, `goal`, `entities`, `constraints`. Ejecuta el loop de aprendizaje sobre esa situación.
- **GET /api/learning/knowledge-base** — Devuelve conceptos, skills y reglas actuales.
- **POST /api/learning/consolidate** — Fuerza una pasada de consolidación.
- **GET /api/learning/uncertainty-status** — Estado del detector de incertidumbre (umbral, fallos por tarea).
- **POST /api/learning/teach-concept** — Body: `concept_name`, `definition`, `properties`, `examples`. Enseñanza humana (humano en el loop).
- **GET /api/learning/growth-metrics** — Métricas de crecimiento: conocimiento, memoria semántica, memoria episódica (si existe), experiencias procesadas, fallos registrados.

Inicialización perezosa: los componentes se crean en la primera llamada a cualquiera de estos endpoints, para no bloquear el arranque si faltan dependencias.

## Flujo típico

1. Llega una situación (p. ej. desde visión o Cursor).
2. Se buscan experiencias similares en memoria semántica.
3. Se calcula confianza y se evalúa incertidumbre.
4. Si hay incertidumbre alta → se consulta al LLM; se extrae guía y conocimiento nuevo y se persiste.
5. Se ejecuta la acción: si se inyectó `action_executor` (p. ej. Cursor o robot real) y la acción no es `ask_for_help`, se llama a ese ejecutor; si no, se usa stub.
6. Se aprende del resultado (éxito/fallo) y se guarda en memoria semántica y en memoria episódica (si está disponible).
7. Periódicamente se ejecuta consolidación para extraer patrones y reglas.

## Ejemplo de uso

```json
POST /api/learning/process-situation
{
  "description": "Veo un objeto cilíndrico rojo brillante",
  "type": "object_identification",
  "goal": "identificar qué es",
  "entities": ["cylindrical_red_object"]
}
```

Respuesta: `action_taken`, `result`, `learned`, `asked_for_help`, `new_knowledge`, `uncertainty_score`.
