# Aprobación: Configuración ANS para ejecutar heals

**Fecha:** 2025-02-13  
**Estado:** Pendiente de aprobación

---

## Resumen

Se configuró `config/atlas.env` para que el ANS (Autonomic Nervous System) **ejecute heals automáticamente** cuando detecte issues, en lugar de quedarse en 0 actions.

---

## Cambios realizados

### 1. `config/atlas.env`

| Variable | Antes | Después | Motivo |
|----------|-------|---------|--------|
| `GOVERNANCE_MODE` | `governed` | `growth` | ANS ejecuta heals sin aprobación manual |
| `ANS_GROWTH_FORCE` | (no existía) | `true` | Bypass adicional si governance falla |
| `ANS_MAX_AUTO_ACTIONS_PER_HOUR` | `10` | `20` | Más capacidad de respuesta por hora |
| `ANS_COOLDOWN_SECONDS` | `60` | `30` | Respuesta más rápida entre heals |
| `ANS_INSTALL_OPTIONAL_DEPS` | (no existía) | `true` | Permite heal `install_optional_deps` (pip) |

### 2. Comentario añadido

```
# growth: ANS ejecuta heals sin aprobación | governed: requiere aprobación
```

---

## Efecto esperado

- **Antes:** "ANS Run Now OK: 5 issues, 0 actions"
- **Después:** El ANS ejecutará heals como `clear_stale_locks`, `restart_scheduler`, `fallback_models`, `install_optional_deps` cuando detecte issues.

---

## Riesgos

- **growth:** El ANS tomará decisiones sin aprobación. Los heals están limitados a `SAFE_HEALS` (clear_stale_locks, restart_scheduler, fallback_models, tune_router, rotate_logs, etc.).
- **ANS_GROWTH_FORCE=true:** Prioriza ejecución sobre governance. Si quieres más control, ponlo en `false` y usa solo `GOVERNANCE_MODE=growth`.

---

## Aprobar / Revertir

### Aprobar
Los cambios ya están en `config/atlas.env`. Reinicia el servicio o el proceso ATLAS para aplicarlos.

### Revertir
Cambiar en `config/atlas.env`:
```
GOVERNANCE_MODE=governed
ANS_GROWTH_FORCE=false
ANS_MAX_AUTO_ACTIONS_PER_HOUR=10
ANS_COOLDOWN_SECONDS=60
# Eliminar ANS_INSTALL_OPTIONAL_DEPS o poner false
```

---

**¿Apruebas estos cambios?** Revisa `config/atlas.env` y confirma.
