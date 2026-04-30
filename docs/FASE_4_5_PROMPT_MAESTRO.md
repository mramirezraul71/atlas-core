# Fase 4-5 - Prompt maestro (inicio)

**Implementado en este commit:** Módulo 4.1 Meta-Learning (MAML).

## Fase 4 - Autonomía superior (Semanas 12-16)

### Módulo 4.1: Meta-Learning (MAML) – Hecho
- `autonomous/learning/meta_learning/maml.py` – MAMLPolicy, MAML (inner_loop, meta_train_step, adapt_to_new_task).
- `autonomous/learning/meta_learning/task_generator.py` – TaskGenerator (navegación, manipulación), estados 128-dim.
- APIs: POST `/api/meta-learning/train`, POST `/api/meta-learning/adapt` (body: MetaAdaptBody), GET `/api/meta-learning/stats`, POST `/api/meta-learning/generate-tasks`.

**Dependencias:** PyTorch (torch), numpy. Opcional: ya en requirements.txt para Fase 2.

### Módulo 4.2: Causal Reasoning – Hecho
- `brain/reasoning/causal_model.py`: CausalGraph (add_variable, add_causal_edge, observe, do_intervention, counterfactual, visualize), CausalReasoner (create_domain, reason_about_action, explain_why).
- APIs: POST `/api/causal/create-domain`, POST `/api/causal/reason`, POST `/api/causal/explain`, POST `/api/causal/counterfactual`.
- Dependencia: networkx>=3.0.

### Pendiente del prompt
- Módulo 4.3: Self-Programming
- Fase 5: JWT, Tests E2E, CI/CD, Observabilidad

*Continuar con el resto del prompt cuando lo pegues.*
