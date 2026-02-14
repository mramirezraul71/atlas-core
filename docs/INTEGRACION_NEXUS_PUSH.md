# Integración ATLAS NEXUS + PUSH

**Documento de arquitectura unificada.**  
Ref: `ATLAS_VISION_E_INTEGRACION.md` (escritorio).

---

## 1. Visión

- **NEXUS** = Robot: dashboard, visión, simulación, control, directivas.
- **PUSH** = Cerebro: orquestador, memoria, aprobaciones, ANS, cluster, gateway.
- **Producto único:** Una plataforma (cuerpo + cerebro) configurable.

---

## 2. Estructura monorepo (ATLAS_PUSH)

```
ATLAS_PUSH/                    ← Raíz (cerebro)
├── atlas_adapter/             ← API HTTP, dashboard unificado
│   └── static/dashboard.html  ← Panel: cerebro + robot (NEXUS)
├── modules/
│   ├── nexus_client.py        ← Cliente para consumir NEXUS
│   └── humanoid/nexus/api.py  ← Router GET /nexus/status
├── nexus/README.md            ← Instrucciones para NEXUS
├── shared/                    ← Tipos comunes (futuro)
└── config/atlas.env           ← NEXUS_ENABLED, NEXUS_BASE_URL
```

---

## 3. Contrato API

### NEXUS expone (cuando corre)

| Endpoint | Descripción |
|----------|-------------|
| `/status` | Estado general |
| `/directives/summary` | Resumen de directivas y proyectos |
| `/directives/health` | Health de directivas |
| `/api/vision/status` o `/vision/status` | Estado de visión |

### PUSH consume (vía `nexus_client`)

- Si `NEXUS_ENABLED=true` y `NEXUS_BASE_URL` configurado:
  - `GET /nexus/status` → llama a NEXUS y agrega estado al panel.
- Si no: el panel muestra "No conectado" con hint de configuración.

---

## 4. Configuración

En `config/atlas.env`:

```env
NEXUS_ENABLED=true
NEXUS_BASE_URL=http://127.0.0.1:8000
NEXUS_TIMEOUT=5
```

### Ejecutar NEXUS (unificado en monorepo)

El código NEXUS está en este repo en `nexus/atlas_nexus/`. Desde la raíz ATLAS_PUSH:

```powershell
python scripts\start_nexus_services.py
# o manualmente:
cd nexus\atlas_nexus
python nexus.py --mode api
```

NEXUS escucha en el puerto configurado (p. ej. 8000). Rutas en `config/atlas.env`: `NEXUS_ATLAS_PATH`, `NEXUS_ROBOT_PATH` apuntan a `ATLAS_PUSH\nexus\...`.

---

## 5. Panel unificado

El dashboard de PUSH incluye:

- **Cerebro (PUSH):** Estado, Versión, Salud, Deploy, ANS, GA, Product, etc.
- **Robot (NEXUS):** Tarjeta "Robot (NEXUS)" con:
  - Conectado / No conectado
  - URL, proyectos, visión (cuando está disponible)

---

## 6. Opciones al usuario

- **Solo PUSH:** NEXUS_ENABLED=false → panel sin NEXUS.
- **PUSH + NEXUS:** NEXUS_ENABLED=true, ejecutar NEXUS → panel unificado.
- **Monorepo:** Código NEXUS ya está en `nexus/atlas_nexus/` y `nexus/atlas_nexus_robot/` dentro de ATLAS_PUSH.

---

## 7. Próximos pasos (opcional)

1. Proxy de comandos: PUSH envía tareas a NEXUS vía API.
2. Stream de visión: integrar en dashboard.
3. Unificar directivas: directivas de NEXUS aplicables desde PUSH.

---

---

## Ensamblado: cerebro conoce y domina

El cerebro expone `GET /brain/assembly` con inventario, estado de cada pieza y rutas de control.  
Ver `C:\ATLAS_NEXUS\ENSAMBLADO_ATLAS.md` para el documento completo de ensamblado.

*Generado como parte de la integración NEXUS + PUSH.*
