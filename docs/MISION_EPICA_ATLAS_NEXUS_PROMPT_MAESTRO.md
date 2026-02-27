# MISIÓN ÉPICA: ATLAS NEXUS - ROBOT MULTIFUNCIONAL WORLD-CLASS
Transformación Completa: Fundación → Percepción → Aprendizaje → Autonomía → Producción

---

## CABECERA

**ROL:** Ingeniero Jefe de ATLAS NEXUS – elevar sistema de 7.2/10 a 9.2/10 world-class.

**CONTEXTO ACTUAL:**
- Repositorio: C:\ATLAS_PUSH (atlas-core)
- Branch: intent-input-rename
- Commit base: 6a1f882
- Estado: Sistema funcional 72/100 con gaps críticos
- Componentes: PUSH (:8791), NEXUS (:8000), Robot (:8002)
- Subsistemas: Health Monitor, Self-Healing, Evolution 2.0, Telemetry, Resilience, Learning

**DIAGNÓSTICO (resumen):**
- Cerebro 88%, Ojos 55% real, Manos 70%, Sangre 65%, Velocidad 60%, Lógica 65%, Motor 85%, Corazón 88%, Pulmones 65%, Esqueleto 82%, Inmune 76%

**GAPS CRÍTICOS:** Sin embeddings, sin world model, visión sin depth/scene, planning reactivo, learning pasivo, sin auth APIs, multi-task limitado, sin meta-learning, reasoning sin explicación, rate limiting inconsistente.

**OBJETIVO FINAL:** Memoria semántica, aprendizaje continuo, programar nuevas skills, predicción de fallos, análisis causal, percepción 3D, razonamiento paso a paso, autonomía tipo NASA, seguridad enterprise.

**VALOR OBJETIVO:** $935K–$1.03M | **NIVEL:** 9.2/10 | **TIEMPO:** 20 semanas

---

## FASE 1: FUNDACIÓN COGNITIVA (Semanas 1-4)

Valor: $100K | Prioridad: CRÍTICA

Objetivo: Memoria semántica, world model predictivo, chain-of-thought explícito.

### MÓDULO 1.1: SEMANTIC MEMORY (Semanas 1-2)

**Problema:** Sin embeddings → búsqueda exacta, no generalización.

**Solución:** Sentence-BERT + FAISS.

**Crear:** `modules/humanoid/memory_engine/semantic_memory.py`

**Clase SemanticMemory:**
- Encoder: sentence-transformers/all-MiniLM-L6-v2 (384 dim)
- Vector store: FAISS IndexFlatIP
- Metadata: SQLite (embeddings_meta.sqlite)
- Persistence: embeddings.faiss + SQLite

**Métodos:**
1. `add_experience(description, context, outcome, metadata)` → id
2. `recall_similar(query, top_k=5, min_similarity=0.6)` → List[Experience]
3. `recall_by_tag(tag, top_k=10)` → List[Experience]
4. `recall_hybrid(query, tags=None, time_range=None, top_k=10)` → List[Experience]
5. `update_experience(id, outcome=None, metadata_update=None)`
6. `delete_experience(id)`
7. `get_statistics()` → Dict

**Integración:**
- Modificar `memory_engine/db.py`: add_plan/add_artifact → también semantic_memory.add_experience; search_similar_plans, search_similar_artifacts
- API PUSH: POST /api/memory/add, POST /api/memory/search, GET /api/memory/stats

**Validación:** Tests de recall semántico; benchmark <50ms en 10K experiences.

**Deps:** `sentence-transformers`, `faiss-cpu`

---

### MÓDULO 1.2: WORLD MODEL (Semana 3)

**Problema:** No predice consecuencias antes de ejecutar.

**Solución:** PyBullet + modelo aprendido (red neuronal).

**Crear:** `brain/world_model/physics_simulator.py` (WorldSimulator)

**Capacidades:**
1. `load_scene(objects)` → scene_id
2. `simulate_action(scene_id, action, steps=240)` → SimulationResult
3. `predict_outcome(current_state, action)` → Prediction
4. `mental_planning(goal, current_state, max_depth=5)` → Plan (MCTS)
5. `learn_from_reality(real_state_before, action, real_state_after)`

**Crear:** `brain/world_model/learned_model.py` (LearnedDynamicsModel)
- Input: state_t, action → Output: delta_state
- predict_next_state, train_batch, save/load checkpoint

**Integración:** logic_engine simula antes de ejecutar; APIs /api/world-model/simulate, /plan, /learn.

**Deps:** `pybullet`, `torch`

---

### MÓDULO 1.3: CHAIN-OF-THOUGHT REASONING (Semana 4)

**Problema:** LLM responde sin mostrar razonamiento.

**Solución:** CoT explícito + self-reflection. Formato obligatorio con `<thinking>` y `<answer>`.

**Modificar:** `nexus/atlas_nexus_robot/backend/brain/reasoning/logic_engine.py`

- Nuevo modo: `cot_enabled=True` (default)
- System prompt fuerza uso de `<thinking>` (pasos 1-6) y `<answer>`, `<confidence>`.

**Capacidades nuevas:**
- `reason_step_by_step(query)` → CoTResult: forzar LLM con tags, parse steps, extraer answer/confidence, guardar chain en memoria. Return `{answer, reasoning_steps, confidence}`.
- `self_reflect(query, answer, reasoning)` → Reflection: LLM critica su razonamiento, assumptions, errores, mejoras.
- `explain_decision(decision_id)` → Explanation: recuperar chain de memoria, explicación legible + confidence.

**Integración:** NEXUS /goal, /think usan CoT; rutas brain del Robot; toggle UI para mostrar/ocultar `<thinking>`.

**API:** POST /api/brain/reason, POST /api/brain/reflect, GET /api/brain/explain/{id}.

**Validación:** Query complejo ≥5 pasos; self_reflect detecta error; explain reconstruye razonamiento.

---

### DELIVERABLES FASE 1

**Archivos creados:**  
`modules/humanoid/memory_engine/semantic_memory.py`, schema `embeddings_meta.sqlite`, `brain/world_model/__init__.py`, `physics_simulator.py`, `learned_model.py`, `mcts_planner.py`, `tests/memory/test_semantic_search.py`, `tests/world_model/test_simulation.py`, `tests/reasoning/test_chain_of_thought.py`.

**Archivos modificados:**  
`memory_engine/db.py`, `logic_engine.py` (CoT), `atlas_http_api.py`, `requirements.txt`.

**Endpoints nuevos:**  
POST /api/memory/add, /api/memory/search, GET /api/memory/stats | POST /api/world-model/simulate, /plan, /learn | POST /api/brain/reason, /api/brain/reflect, GET /api/brain/explain/{id}.

**Tests:** 15+ unitarios, 5+ integración, benchmarks. **Valor:** $100K | **Tiempo:** 4 semanas.

---

## FASE 2: PERCEPCIÓN AVANZADA (Semanas 5-7)

Valor: $100K | Prioridad: ALTA. Objetivo: Visión 3D + comprensión de escena semántica.

### MÓDULO 2.1: MONOCULAR DEPTH ESTIMATION (Semana 5)

**Problema:** YOLO no da distancia (2m vs 10m). **Solución:** MiDaS v3.1 (DPT-Hybrid 384x384).

**Crear:** `nexus/atlas_nexus_robot/backend/vision/depth_estimation.py` — Clase `DepthEstimator`.

**Capacidades:** `estimate_depth(frame)`, `get_distance_at_point(depth_map, x, y)`, `get_distance_in_bbox(depth_map, bbox)`, `calibrate_metric(...)`, `generate_point_cloud(frame, depth_map, camera_intrinsics)`.

**Integración:** Tras YOLO, depth por objeto; campo `distance_m`/`distance_relative`. API: POST /api/vision/depth/estimate, /api/vision/depth/pointcloud. **Deps:** timm, MiDaS torch hub.

### MÓDULO 2.2: SCENE UNDERSTANDING con VLM (Semana 5)

**Problema:** Solo lista de objetos, no "qué está pasando". **Solución:** LLaVA 1.6 (Ollama) o CLIP+GPT.

**Crear:** `vision/scene_understanding.py` — Clase `SceneUnderstanding`.

**Capacidades:** `describe_scene(frame, detail_level)`, `answer_visual_question(frame, question)`, `detect_anomalies(frame, baseline_description)`, `categorize_scene(frame)`.

**Integración:** /api/vision/scene/describe, /api/vision/scene/ask; logic_engine usa descripción como contexto. **Deps:** Ollama llava o transformers.

### MÓDULO 2.3: MULTI-CAMERA SYSTEM (Semanas 6-7)

**Objetivo:** 4-6 cámaras con fusión. Layout: Front, Left, Right, Back (opcional Top).

**Crear:** `vision/multi_camera_manager.py` (MultiCameraManager), `vision/camera_fusion.py` (CameraFusion).

**MultiCameraManager:** `register_camera`, `get_camera_frame`, `get_all_frames`, `get_360_view`, `get_stereo_depth`, `detect_object_in_any_camera`, `get_best_view_of_object`, `health_check`.

**CameraFusion:** `fuse_detections`, `triangulate_3d_position`, `create_occupancy_map`.

**API:** /api/vision/cameras/list, /api/vision/cameras/{id}/stream, /api/vision/cameras/all/detections, /api/vision/360. Dashboard multi-cámara en frontend.

**Deliverables Fase 2:** depth_estimation.py, scene_understanding.py, multi_camera_manager.py, camera_fusion.py, calibration.py + tests. **Valor:** $100K | **Tiempo:** 3 semanas.

---

## FASE 3: APRENDIZAJE ACTIVO (Semanas 8-11)

Valor: $155K | Prioridad: ALTA. Objetivo: Aprendizaje activo (curiosity) + replay + RL para tools.

### MÓDULO 3.1: ACTIVE LEARNING con Curiosity (Semanas 8-9)

**Crear:** `autonomous/learning/active_learner.py` (ActiveLearner), `uncertainty_ensemble.py` (UncertaintyEnsemble).

**ActiveLearner:** `estimate_uncertainty(state)`, `generate_curious_queries()`, `plan_exploration_episode(duration_minutes)`, `execute_and_learn(exploration_plan)`, `get_knowledge_gaps()`.

**UncertaintyEnsemble:** 5 modelos, `predict_with_uncertainty`, `calibrate_uncertainty`. Background: exploración diaria/downtime.

**API:** GET /api/learning/knowledge-gaps, POST /api/learning/explore, GET /api/learning/uncertainty/{task}.

### MÓDULO 3.2: EPISODIC REPLAY (Semana 10)

**Crear:** `autonomous/learning/episodic_replay.py` — Clase `EpisodicReplay`.

**Capacidades:** `store_experience`, `sample_batch(strategy)`, `offline_learning_session`, `dream_planning(goal)`, `consolidate_knowledge()`. Background 2AM/3AM.

**API:** POST /api/learning/replay/start, GET /api/learning/replay/stats.

### MÓDULO 3.3: TOOL USE RL (Semana 11)

**Crear:** `autonomous/learning/tool_use_rl.py` — Clase `ToolUseRL` (PPO). State/action/reward para tool selection.

**Capacidades:** `select_tool_and_params(task, context, tools)`, `execute_and_learn`, `train_from_demonstrations`, `get_tool_usage_stats`, `explain_tool_choice`. Integración en tools_registry y NEXUS /goal.

**API:** POST /api/tools/rl/select, /api/tools/rl/feedback, GET /api/tools/rl/stats.

**Deliverables Fase 3:** active_learner, uncertainty_ensemble, episodic_replay, tool_use_rl, rl_policy + tests. Background: 2AM offline, 3AM consolidate, exploration en downtime. **Valor:** $155K | **Tiempo:** 4 semanas.

---

## FASE 4: AUTONOMÍA SUPERIOR (Semanas 12-16)

Valor: $285K | Prioridad: INNOVACIÓN MUNDIAL.  
Objetivo: Meta-learning, razonamiento causal, self-programming (diferenciación mundial).  
*[CONTINUARÁ EN SIGUIENTE MENSAJE - especificación detallada pendiente]*

---

## FASE 5: SEGURIDAD, TESTS, CI/CD, DOCS

*Pendiente de especificación.*

---

## RESUMEN ACUMULADO

| Fase | Contenido | Valor | Semanas |
|------|-----------|-------|---------|
| 1 | Memoria semántica + World model + Chain-of-thought | $100K | 1-4 |
| 2 | Depth + Scene understanding + Multi-camera | $100K | 5-7 |
| 3 | Active learning + Episodic replay + Tool RL | $155K | 8-11 |
| 4 | Meta-learning, Causal, Self-code | $285K | 12-16 (spec pendiente) |
| 5 | Seguridad, Tests, CI/CD, Docs | — | 17-20 (spec pendiente) |

**Valor especificado hasta ahora:** $355K (Fases 1-3). **Tiempo:** 11 semanas. **Faltan:** detalle Fase 4, Fase 5 completa.
