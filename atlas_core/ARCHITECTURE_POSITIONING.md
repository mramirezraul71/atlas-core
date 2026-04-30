# ATLAS Core - Posicionamiento Canónico

## Rol arquitectónico

`atlas_core` es el núcleo canónico del sistema ATLAS y vive en la raíz del repositorio:

- `C:\ATLAS_PUSH\atlas_core`

`atlas_code_quant` es un módulo consumidor del núcleo. No aloja la implementación real de `atlas_core`.

## ATLAS como organismo biológico

- **Cerebro**: `atlas_core/brain` (`brain_core.py`, `safety_kernel.py`, `arbitration.py`, `mission_manager.py`)
- **Corazón**: `brain_audit_log.jsonl` (latidos de decisión, bloqueos, autorizaciones y fail-safe)
- **Sistema nervioso**: `atlas_core/runtime` (`heartbeat_loop.py`, `event_loop.py`, `mode_manager.py`)
- **Sistema inmune**: `safety_kernel.py` + `adapters/healing_adapter.py`
- **Memoria**: `audit_log.py`, estado en `data/operation`, y trazas de ejecución del `StateBus`
- **Manos**: `atlas_code_quant` ejecuta, pero no gobierna

## Ciclo fisiológico (cada pocos segundos)

1. Heartbeat consulta adapters y actualiza estado/riesgo.
2. Event loop procesa eventos y arbitra comandos.
3. Safety kernel autoriza o bloquea.
4. WorkspaceBridge publica intención y resultado.
5. Command router ejecuta.
6. Se valida evidencia; si hay anomalía, dispara healing cascade.
7. Audit log registra cadena causal completa.

## Flujo de señales (percepción -> decisión -> acción -> aprendizaje)

1. Percepción (`vision`, `system_health`, `quant`) -> `StateBus`.
2. Decisión (`BrainCore`) -> arbitraje + política + seguridad.
3. Acción (`CommandRouter` + adapters).
4. Aprendizaje operativo (`AuditLog`, métricas en `StateBus`, evidencia de ejecución).

## Autorregulación (auto-curación)

- Si `SafetyKernel` detecta riesgo crítico, activa `fail_safe` y fuerza modo seguro.
- Si `WorkspaceBridge` detecta anomalía de ejecución, dispara cascada de curación:
  - `healing.diagnose`
  - `healing.recover`
- Toda transición queda auditada para diagnóstico posterior.

## Diagrama ASCII de arquitectura neurológica

```text
                  ┌──────────────────────────────────────────┐
                  │               Workspace                  │
                  │      (supervisión y feedback humano)     │
                  └───────────────────┬──────────────────────┘
                                      │
                                      ▼
┌──────────────┐    eventos    ┌───────────────┐   comandos   ┌────────────────┐
│   Adapters   │──────────────▶│   BrainCore   │─────────────▶│ CommandRouter  │
│(visión/quant │               │(arbitra/decide│              │ (ejecución)    │
│/body/salud)  │◀──────────────│/audita)       │◀─────────────│                │
└──────┬───────┘   estado/riesgo└──────┬────────┘   resultados  └──────┬─────────┘
       │                                │                               │
       ▼                                ▼                               ▼
┌──────────────┐                 ┌──────────────┐                 ┌──────────────┐
│   StateBus   │                 │ SafetyKernel │                 │   Healing    │
│(memoria viva)│                 │(inmunidad)   │                 │ (diagnose/   │
└──────┬───────┘                 └──────┬───────┘                 │  recover)    │
       │                                │                         └──────────────┘
       ▼                                ▼
┌──────────────┐                 ┌──────────────┐
│ HeartbeatLoop│                 │  AuditLog    │
│ EventLoop    │                 │ (corazón)    │
└──────────────┘                 └──────────────┘
```

## Compatibilidad temporal (legacy shim)

Se mantiene shim mínimo en:

- `C:\ATLAS_PUSH\atlas_code_quant\atlas_core`

Ese shim solo redirige imports al `atlas_core` raíz. Implementación real: solo en `C:\ATLAS_PUSH\atlas_core`.
