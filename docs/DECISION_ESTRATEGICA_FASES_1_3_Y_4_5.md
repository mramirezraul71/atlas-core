# Decisión estratégica – Estado Fases 1-3 y opciones 4-5

## Estado actual

### 1. Tests de memoria (3 skipped)
- **Solución aplicada:** `@pytest.mark.skipif(SKIP_TESTS, reason="Dependencies missing: ...")` con probe de import + instanciación en `tests/memory/test_semantic_search.py`.
- Si persiste el skip: PyTorch>=2.4 + sentence-transformers>=3.0 + datasets>=2.14 + huggingface-hub>=0.19.

### 2. World model (solo stub)
- **Estado:** API `/api/world-model/simulate` responde pero no simula.
- **Impacto:** Sin predicción física, sin mental planning, sin validación de acciones antes de ejecutar.
- **Prioridad:** ALTA (valor ~$85K).
- **Opciones:** A) Completo ahora (4-5 días) | B) Continuar Fases 4-5 y volver después | C) Versión simplificada (2 días).

### 3. Chain-of-Thought (pendiente)
- **Estado:** Sin modificar `logic_engine.py`.
- **Impacto:** Razonamiento black-box, sin explicaciones, debugging difícil.
- **Prioridad:** MEDIA (valor ~$25K).
- **Estimación:** ~1 día.

### 4. Fase 3 stubs
- **Estado:** Active Learning, Episodic Replay, Tool RL = TODOs (esperado).
- **Acción:** Implementar en siguiente iteración.

---

## Opciones estratégicas

### OPCIÓN A: Completar Fases 1-3 (recomendada)
- Arreglar tests memoria, World Model simplificado (2 días), CoT (1 día), Fase 3 stubs después.
- **Resultado:** Cognición y percepción sólidas; base para 4-5. Valor ~$120K → ~$230K. **Tiempo:** 3-4 días.

### OPCIÓN B: Saltar a Fases 4-5 (agresiva)
- Arreglar tests memoria, ir directo a Meta-Learning y Auth, World Model después.
- **Resultado:** Innovaciones y seguridad primero; gaps en fundación. **Tiempo:** 9 semanas.

### OPCIÓN C: Validar y estabilizar (conservadora)
- Todos los tests, benchmarks, E2E, documentación (5 días). Sin nuevas features.
- **Resultado:** Sistema ultra-estable, production-ready.

---

## Recomendación: Opción A + validación parcial

```
DÍA 1-2: Arreglar tests memoria | World Model simplificado (reglas físicas básicas, predicción simple, API funcional) | Tests World Model
DÍA 3:   Chain-of-Thought en logic_engine | API /api/brain/reason | Tests CoT
DÍA 4:   Validación E2E | Benchmarks básicos | Documentación actualizada

RESULTADO: Fase 1 y 2 100% funcional; Fase 3 stubs OK; valor ~$230K; base sólida para Fases 4-5.
```

---

## Fases 4-5 (resumen preparado)

**FASE 4 – Autonomía superior**
- Meta-Learning (MAML) ~$180K
- Causal Reasoning ~$75K
- Self-Programming ~$120K

**FASE 5 – Producción**
- JWT Authentication ~$30K
- Tests E2E ~$30K
- CI/CD ~$40K
- Observabilidad ~$50K

**Total adicional:** ~$525K | **Tiempo:** 9 semanas.

---

*Documento para decisión; prompt detallado de Opción A o Fases 4-5 disponible cuando lo indiques.*
