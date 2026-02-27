# Integrar aquí el código de ATLAS NEXUS (unificación monorepo)

Esta carpeta es el **destino** del código de **ATLAS NEXUS** dentro del repo **atlas-core** (ATLAS_PUSH).

---

## Pasos

### 1. Crear estructura

En la raíz del repo (ATLAS_PUSH):

```
nexus/
├── atlas_nexus/          ← Copiar aquí TODO el contenido de C:\ATLAS_NEXUS\atlas_nexus
│   └── nexus.py          ← Debe existir; arranque: python nexus.py --mode api
└── atlas_nexus_robot/    ← Opcional: copiar C:\ATLAS_NEXUS\atlas_nexus_robot
    └── backend/          ← Contenido de atlas_nexus_robot\backend
```

### 2. Copiar desde tu máquina

**NEXUS API (obligatorio):**

```powershell
# Desde PowerShell, en la raíz del repo ATLAS_PUSH
$src = "C:\ATLAS_NEXUS\atlas_nexus"
$dst = "C:\ATLAS_PUSH\nexus\atlas_nexus"
New-Item -ItemType Directory -Force -Path $dst | Out-Null
Copy-Item -Path "$src\*" -Destination $dst -Recurse -Force
```

**Robot backend (opcional):**

```powershell
$srcRobot = "C:\ATLAS_NEXUS\atlas_nexus_robot\backend"
$dstRobot = "C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend"
New-Item -ItemType Directory -Force -Path $dstRobot | Out-Null
Copy-Item -Path "$srcRobot\*" -Destination $dstRobot -Recurse -Force
```

### 3. Configurar rutas en este repo

En `config/atlas.env`:

```env
NEXUS_ATLAS_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus
NEXUS_ROBOT_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend
```

(Ajusta `C:\ATLAS_PUSH` si tu ruta del repo es otra.)

### 4. Verificar

- Desde `ATLAS_PUSH`: `python nexus\atlas_nexus\nexus.py --mode api` (o usar `scripts\start_nexus_services.py` tras cargar atlas.env).
- PUSH debe seguir conectando a NEXUS con `NEXUS_BASE_URL=http://127.0.0.1:8000` y el dashboard unificado debe mostrar estado del robot.

---

Véase **docs/UNIFICACION_ATLAS_NEXUS_PUSH.md** para el plan completo y el checklist de unificación.
