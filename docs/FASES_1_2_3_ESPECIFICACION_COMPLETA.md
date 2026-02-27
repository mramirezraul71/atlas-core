# MISIÓN: ATLAS NEXUS WORLD-CLASS - FASES 1-3
Fundación Cognitiva + Percepción Avanzada + Aprendizaje Activo

---

**ROL:** Ingeniero Jefe de ATLAS NEXUS  
**OBJETIVO:** Implementar 11 semanas de desarrollo en 3 fases secuenciales  
**REPOSITORIO:** C:\ATLAS_PUSH (atlas-core)  
**BRANCH:** intent-input-rename  
**COMMIT BASE:** 6a1f882  

**CONTEXTO:**  
Sistema robótico funcional 72/100 con gaps críticos en: memoria sin embeddings, sin world model, visión 2D sin depth/scene, learning pasivo, planning reactivo.

**TRANSFORMACIÓN:** 72/100 → 85/100 (Fases 1-3) | $145K-$240K → $500K-$600K | 7.2/10 → 8.5/10

---

## FASE 1: FUNDACIÓN COGNITIVA (Semanas 1-4)
Valor: $100K | Archivos: 15 nuevos, 5 modificados

### MÓDULO 1.1: SEMANTIC MEMORY (Semanas 1-2)
CRÍTICO: Informe "embeddings-pendiente"

**Crear:** `modules/humanoid/memory_engine/semantic_memory.py`

- Clase `SemanticMemory`: encoder `all-MiniLM-L6-v2`, FAISS IndexFlatIP, SQLite metadata.
- Métodos: `add_experience`, `recall_similar`, `get_statistics` (y según spec extendida: `recall_by_tag`, `recall_hybrid`, `update_experience`, `delete_experience`).
- Persistencia: `embeddings.faiss` + SQLite (experiences con embedding_index).

**Modificar:** `modules/humanoid/memory_engine/db.py` / `store.py`
- Añadir integración: al guardar plan (store_plan) → también `semantic_memory.add_experience` con tag `plan`.
- Nuevo: `search_similar_plans(query, top_k)` usando `semantic_memory.recall_similar(..., tags_filter=['plan'])`.

**API en** `atlas_adapter/atlas_http_api.py`:
- `POST /api/memory/add` → add_experience
- `POST /api/memory/search` → recall_similar
- `GET /api/memory/stats` → get_statistics

**Dependencias:** `sentence-transformers`, `faiss-cpu` (añadir a requirements.txt).

---

### MÓDULO 1.2: WORLD MODEL (Semana 3)
- PyBullet + learned model. Archivos: `brain/world_model/physics_simulator.py`, `learned_model.py`, `mcts_planner.py`.
- APIs: /api/world-model/simulate, /plan, /learn.

### MÓDULO 1.3: CHAIN-OF-THOUGHT (Semana 4)
- Modificar logic_engine: `reason_step_by_step`, `self_reflect`, `explain_decision`.
- APIs: /api/brain/reason, /api/brain/reflect, /api/brain/explain/{id}.

---

## FASE 2: PERCEPCIÓN AVANZADA (Semanas 5-7)
- Depth (MiDaS), Scene Understanding (LLaVA/CLIP), Multi-Camera + CameraFusion.

## FASE 3: APRENDIZAJE ACTIVO (Semanas 8-11)
- Active Learner + Uncertainty Ensemble, Episodic Replay, Tool Use RL.

---

*Contenido pegado por el usuario; código detallado del Módulo 1.1 implementado en el repo. Resto de fases según MISION_EPICA_ATLAS_NEXUS_PROMPT_MAESTRO.md.*
