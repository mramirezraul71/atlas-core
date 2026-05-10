# ATLAS T4 Robot — Documentación

Documentación del robot autónomo de futuros sobre Plus500 Futures Technologies (T4) integrado con `atlas-core`.

## Índice

1. [Informe Fase 0 (auditoría + plan por fases)](./00_INFORME_FASE0.md)
2. [Arquitectura (C4 + ADRs)](./01_ARQUITECTURA.md)
3. [Contratos de datos](./02_CONTRATOS_DE_DATOS.md)
4. [API tiempo real (REST + WS + SSE)](./03_API_TIEMPO_REAL.md)
5. [Paridad Paper/Live](./04_PARIDAD_PAPER_LIVE.md)
6. [Riesgo y sizing](./05_RIESGO_Y_SIZING.md)
7. [Tests y QA](./06_TESTS_Y_QA.md)
8. [Guía de build & run](../T4_ROBOT_BUILD.md)

## Diagramas (Mermaid)

- [C4 Context](./diagrams/C4_context.mmd)
- [C4 Containers](./diagrams/C4_container.mmd)
- [Pipeline secuencia](./diagrams/pipeline_sequence.mmd)
- [Live unlock states](./diagrams/live_unlock_state.mmd)

## Estado actual

**Fase 0 — Auditoría + Plan**. Documentos listos para revisión. No hay código nuevo (servicio + dashboard) todavía. La integración real con credenciales **no** se inicia hasta aprobación explícita del informe.

## Decisiones cerradas (10 May 2026)

| Tema | Decisión |
|---|---|
| Alcance regulatorio | Cuenta T4 US disponible; diseño contra WS+Protobuf real, paper en T4 Simulator |
| Integración con atlas-core | Servicio externo + bridge (Redis pub/sub) |
| Dashboard | Híbrido: SPA Next.js operativa + Grafana para series históricas |
| Secretos | Doppler/Infisical (SaaS); `.env` solo bootstrap |

## Branch

Trabajo Fase 0: `feat/t4-robot-phase0` (deriva de `intent-input-rename`).
