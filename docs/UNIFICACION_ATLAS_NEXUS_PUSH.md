# Unificación ATLAS NEXUS + ATLAS PUSH en un solo repo (atlas-core)

**Objetivo:** Traer el código de **ATLAS NEXUS** (robot, visión, directivas) dentro del repositorio **atlas-core** (ATLAS_PUSH) para tener un único monorepo: cerebro + cuerpo.

---

## 1. Situación actual

| Componente | Ubicación actual | Rol |
|------------|------------------|-----|
| **PUSH (cerebro)** | Este repo `ATLAS_PUSH` → GitHub `mramirezraul71/atlas-core` | Orquestador, memoria, ANS, dashboard, gateway |
| **NEXUS (robot)** | `C:\ATLAS_NEXUS\atlas_nexus` (repo/carpeta externa) | API status/directivas/visión, dashboard robot, cámaras |
| **Robot frontend/API** | `C:\ATLAS_NEXUS\atlas_nexus_robot\backend` (y frontend en 5174) | Cámaras, visión, control |

PUSH ya consume NEXUS vía `NEXUS_BASE_URL` y `nexus_client.py`; el dashboard unificado está en `atlas_adapter/static/dashboard.html`. La unificación consiste en **mover el código de NEXUS dentro de este repo** y ajustar rutas y config.

---

## 2. Estructura objetivo (monorepo)

```
ATLAS_PUSH/                          ← Raíz = atlas-core (GitHub)
├── atlas_adapter/                   ← API HTTP PUSH, dashboard unificado (ya existe)
├── config/atlas.env                 ← NEXUS_* y rutas apuntando a nexus/ (ajustar)
├── docs/
│   ├── INTEGRACION_NEXUS_PUSH.md    ← Arquitectura (ya existe)
│   └── UNIFICACION_ATLAS_NEXUS_PUSH.md  ← Este doc
├── modules/
│   ├── nexus_client.py              ← Cliente NEXUS (ya existe)
│   ├── nexus_proxy.py               ← Proxy /robot/ (ya existe)
│   └── humanoid/nexus/api.py        ← GET /nexus/status (ya existe)
├── nexus/                           ← Código NEXUS unificado (destino)
│   ├── README.md                    ← Instrucciones (ya existe, ampliado)
│   ├── INTEGRAR_AQUI.md             ← Checklist “pegar aquí desde ATLAS_NEXUS”
│   ├── atlas_nexus/                 ← Copiar aquí: C:\ATLAS_NEXUS\atlas_nexus
│   │   └── nexus.py                 ← Punto de entrada: python nexus.py --mode api
│   └── atlas_nexus_robot/           ← Opcional: copiar backend/frontend robot
│       ├── backend/                 ← C:\ATLAS_NEXUS\atlas_nexus_robot\backend
│       └── frontend/                ← Si aplica (Vite/React en 5174)
├── scripts/
│   ├── start_atlas.ps1              ← Arrancar PUSH (ya existe)
│   └── start_nexus_services.py      ← Arrancar NEXUS desde nexus/ (ajustar rutas)
├── shared/                          ← Tipos comunes (ya existe)
└── run_atlas_core.py                ← Entrada principal PUSH (ya existe)
```

---

## 3. Checklist de unificación

### Fase A — Preparar en este repo (ATLAS_PUSH)

- [ ] Crear `nexus/atlas_nexus/` (vacío o con `.gitkeep`) como destino.
- [ ] Crear `nexus/atlas_nexus_robot/` si se unifica también el robot (backend/frontend).
- [ ] Tener este documento y `nexus/INTEGRAR_AQUI.md` listos.

### Fase B — Copiar código desde ATLAS_NEXUS

- [ ] Copiar **todo** el contenido de `C:\ATLAS_NEXUS\atlas_nexus` → `ATLAS_PUSH\nexus\atlas_nexus\`.
- [ ] Verificar que existe `nexus/atlas_nexus/nexus.py` y que arranca con `python nexus.py --mode api`.
- [ ] Si se unifica el robot: copiar `C:\ATLAS_NEXUS\atlas_nexus_robot\backend` → `nexus/atlas_nexus_robot/backend` (y frontend si aplica).

### Fase C — Ajustar configuración

- [ ] En `config/atlas.env`:
  - `NEXUS_ATLAS_PATH` = `C:\ATLAS_PUSH\nexus\atlas_nexus` (o ruta relativa si usas una desde raíz).
  - `NEXUS_ROBOT_PATH` = `C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend` (si unificaste robot).
- [ ] Revisar `scripts/start_nexus_services.py`: que use `NEXUS_ATLAS_PATH` (o la nueva ruta) para arrancar NEXUS.
- [ ] Revisar ANS: `modules/humanoid/ans/heals/restart_nexus_services.py` y `nexus_services_health.py` para que usen las mismas rutas.

### Fase D — Imports y dependencias

- [ ] Si NEXUS tiene imports absolutos que asumen `C:\ATLAS_NEXUS`, ajustarlos para que funcionen desde `ATLAS_PUSH\nexus\atlas_nexus` (por ejemplo con `sys.path` o PYTHONPATH desde el script de arranque).
- [ ] Añadir al repo un `requirements-nexus.txt` o sección en el README con dependencias específicas de NEXUS (si las hay), para no romper el entorno PUSH.

### Fase E — Documentación y commit

- [ ] Actualizar `docs/INTEGRACION_NEXUS_PUSH.md`: indicar que NEXUS ya vive en este repo y que la ejecución es desde `nexus/atlas_nexus`.
- [ ] Actualizar `nexus/README.md` con la nueva estructura y comandos desde raíz del repo.
- [ ] Commit: “Unificación NEXUS en monorepo atlas-core” (solo después de verificar que PUSH + NEXUS arrancan y el dashboard sigue funcionando).

---

## 4. Rutas que debes cambiar después de copiar

| Variable / archivo | Antes | Después (ejemplo) |
|-------------------|--------|-------------------|
| `NEXUS_ATLAS_PATH` (atlas.env) | `C:\ATLAS_NEXUS\atlas_nexus` | `C:\ATLAS_PUSH\nexus\atlas_nexus` |
| `NEXUS_ROBOT_PATH` (atlas.env) | `C:\ATLAS_NEXUS\atlas_nexus_robot\backend` | `C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend` |
| start_nexus_services.py | Usa env o ruta fija a ATLAS_NEXUS | Usar `NEXUS_ATLAS_PATH` / raíz del repo |
| restart_nexus_services (ANS) | Idem | Idem |

---

## 5. Comandos de arranque unificados (objetivo)

Desde la raíz del repo `ATLAS_PUSH`:

```powershell
# Opción 1: Solo cerebro (PUSH)
.\scripts\start_atlas.ps1

# Opción 2: Cerebro + NEXUS (cuando NEXUS esté bajo nexus/)
$env:NEXUS_ATLAS_PATH = "C:\ATLAS_PUSH\nexus\atlas_nexus"
python scripts\start_nexus_services.py   # en otra terminal o integrado en start_atlas.ps1
.\scripts\start_atlas.ps1
```

---

## 6. Resumen

- **Sí, se trata de unificar Atlas Nexus con Atlas Push en el mismo repo (atlas-core).**
- En este repo ya está la parte “cerebro” (PUSH) y la integración (nexus_client, dashboard, proxy). Falta **traer el código de NEXUS** a la carpeta `nexus/` y **actualizar rutas y scripts** según este plan.
- Los archivos “de preparación” son: este documento y `nexus/INTEGRAR_AQUI.md` (checklist concreto para pegar el código de ATLAS_NEXUS).

*Documento generado para la unificación NEXUS + PUSH en atlas-core.*
