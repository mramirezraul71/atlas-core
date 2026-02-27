# Arranque de NEXUS (puerto 8000)

Para que el dashboard muestre **NEXUS (8000): Conectado**, el servicio NEXUS debe estar en ejecución en el puerto 8000.

## Opción 1: Desde el dashboard (recomendado)

1. Abre el dashboard PUSH: `http://127.0.0.1:8791/ui`
2. En la tarjeta **Estado**, pulsa **Reconectar NEXUS**.
3. Espera unos segundos; el estado pasará a "Conectado" cuando NEXUS responda en 8000.

## Opción 2: Manual (PowerShell)

Desde la raíz del repo (`C:\ATLAS_PUSH`):

```powershell
.\scripts\start_nexus.ps1
```

O desde la carpeta de NEXUS:

```powershell
cd nexus\atlas_nexus
.\start.ps1 -mode api
```

- Si existe `venv`, se usará. Si no, se usará el Python del sistema.
- API: `http://localhost:8000`, docs: `http://localhost:8000/docs`.

## Configuración

En `config/atlas.env`:

- `NEXUS_ENABLED=true`
- `NEXUS_BASE_URL=http://127.0.0.1:8000`
- `NEXUS_ATLAS_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus` (ajusta si el repo está en otra ruta)

Si el puerto 8000 está ocupado, puedes ejecutar antes `.\scripts\free_port_8000.ps1 -Kill` para liberarlo.

## Matar proceso + limpieza de caché (trabajo en navegador)

Al usar el dashboard en el navegador, conviene **matar el proceso pendiente** y **limpiar caché** antes de arrancar de nuevo:

1. **Desde el dashboard**: al pulsar **Reconectar NEXUS** se libera el puerto 8000, se limpia caché del servidor y se arranca NEXUS. El botón **Limpiar caché** limpia localStorage/sessionStorage del navegador y la caché del servidor (`__pycache__`, `temp_models_cache`), luego refresca los datos.
2. **Script unificado** (terminal): desde la raíz del repo:
   - `.\scripts\restart_service_clean.ps1 -Service nexus -ClearCache` — mata proceso en 8000, limpia caché y arranca NEXUS.
   - `.\scripts\restart_service_clean.ps1 -Service robot -ClearCache` — igual para Robot (8002).
   - `.\scripts\restart_service_clean.ps1 -Service all -ClearCache` — libera 8000, 8002 y 8791; limpia caché; arranca NEXUS y Robot.

Puerto genérico: `.\scripts\free_port.ps1 -Port 8000 -Kill` (o 8791, 8002).
