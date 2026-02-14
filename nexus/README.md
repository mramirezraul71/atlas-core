# NEXUS — Robot (ATLAS NEXUS)

Módulo de integración entre **ATLAS PUSH** (cerebro) y **ATLAS NEXUS** (robot). En **monorepo**, el código de NEXUS vive aquí bajo `nexus/`.

## Contrato

- **NEXUS** expone: `/status`, `/directives/summary`, `/directives/health`, `/api/vision/status`, etc.
- **PUSH** consume vía `modules/nexus_client.py` cuando `NEXUS_BASE_URL` está configurado.
- Panel unificado: estado del robot + estado del cerebro en un solo dashboard.

## Acceso unificado (mismo puerto)

PUSH expone **`/robot/`** como proxy a NEXUS. Desde el dashboard (8791), el botón **"Ver Robot"** abre NEXUS en `http://127.0.0.1:8791/robot/` — sin cambiar de puerto.

## Configuración

En `config/atlas.env`:

```env
NEXUS_ENABLED=true
NEXUS_BASE_URL=http://127.0.0.1:8000
# Tras unificación en monorepo (opcional):
NEXUS_ATLAS_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus
NEXUS_ROBOT_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend
```

## Ejecutar NEXUS

**Si NEXUS ya está unificado en este repo** (código en `nexus/atlas_nexus/`):

```powershell
# Desde la raíz ATLAS_PUSH (cargar atlas.env antes si hace falta)
python scripts\start_nexus_services.py
# o manualmente:
cd nexus\atlas_nexus
python nexus.py --mode api
```

**Si NEXUS sigue en carpeta externa:**

```powershell
cd C:\ATLAS_NEXUS\atlas_nexus
python nexus.py --mode api
```

NEXUS escuchará en el puerto configurado (p. ej. 8000).

## Estructura (monorepo)

```
ATLAS_PUSH/
├── nexus/                        ← Código NEXUS unificado
│   ├── README.md                 ← Este archivo
│   ├── INTEGRAR_AQUI.md         ← Cómo copiar ATLAS_NEXUS aquí
│   ├── atlas_nexus/             ← Contenido de C:\ATLAS_NEXUS\atlas_nexus
│   └── atlas_nexus_robot/       ← Backend/frontend robot (opcional)
├── modules/nexus_client.py
├── modules/humanoid/nexus/api.py   ← GET /nexus/status
├── shared/                        ← Tipos comunes
└── atlas_adapter/                 ← API HTTP, dashboard unificado
```

---

- **Arquitectura:** `docs/INTEGRACION_NEXUS_PUSH.md`
- **Unificación en un solo repo:** `docs/UNIFICACION_ATLAS_NEXUS_PUSH.md` y `nexus/INTEGRAR_AQUI.md`
