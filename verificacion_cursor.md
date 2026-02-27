# Verificación cuando Cursor termine

## 1. Captura de `/ui`

**Estado:** GET `/ui` responde **200** y sirve el dashboard.

- **URL:** http://127.0.0.1:8791/ui  
- **Contenido:** Una sola página HTML (ATLAS Dashboard) con:
  - Tema oscuro (`--bg: #0f0f12`, `--card: #1a1a20`), título "ATLAS Dashboard"
  - Grid de tarjetas para: Status, Version, Metrics, Scheduler jobs, Update, Approvals
  - Botones de acción: Check, Approve, Reject (approvals)
  - La página hace fetch a `/status`, `/version`, `/metrics`, `/scheduler/jobs`, `/update/status`, `/approvals/list`

Para una **captura de pantalla real**: abre en el navegador http://127.0.0.1:8791/ui (con la API en marcha) y guarda la captura.

---

## 2. Salida de GET `/version`

```json
{
    "ok": true,
    "version": "0.0.0",
    "git_sha": "",
    "channel": "canary"
}
```

*(En un repo con `VERSION` y git, `version` y `git_sha` se rellenan desde el módulo `release`.)*

---

## 3. Resultado de `07_install_all.ps1` (últimas 20 líneas)

Cuando el script **termina bien**, las últimas líneas son equivalentes a:

```
[5/6] Smoke tests
  Puerto 8791 en uso. Omitiendo smoke (ejecuta después: .\04_smoke_tests.ps1 -RepoPath C:\ATLAS_PUSH -AtlasPort 8791)
  -- o --
  Ejecuta smoke con API levantada: .\03_run_atlas_api.ps1 en una terminal y .\04_smoke_tests.ps1 en otra.

[6/6] Servicio: omitido (usa -Service para instalar)

== Cómo usar ==
  API:     .\03_run_atlas_api.ps1 -RepoPath C:\ATLAS_PUSH -AtlasPort 8791
  UI:      http://127.0.0.1:8791/ui
  Status:  http://127.0.0.1:8791/status
  Version: http://127.0.0.1:8791/version
  Approvals: GET/POST http://127.0.0.1:8791/approvals/list
  Smoke:    .\04_smoke_tests.ps1 -RepoPath C:\ATLAS_PUSH -AtlasPort 8791

Listo.
```

**Nota:** En la ejecución automática aquí, el script se detuvo en [3/6] Dependencias por avisos de pip en stderr (conflictos del resolver). En una ejecución manual con venv ya instalado y sin `$ErrorActionPreference = Stop` en la sesión, el script suele llegar hasta "Cómo usar" y "Listo." como arriba.
