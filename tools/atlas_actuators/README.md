# ATLAS Actuators (Generic)

Infraestructura base de actuadores para ATLAS:
- `atlas_vision_capture.js`: capturas visuales (Playwright, Puppeteer o escritorio).
- `atlas_action_trigger.js`: ejecucion de acciones web controladas por disparadores.
- `atlas_actuator_healthcheck.js`: sanidad de dependencias + validacion `TASK_OK`.

## Requisitos
- Node.js 18+
- Variable de entorno `ATLAS_CENTRAL_CORE`

## Instalacion
```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\install_atlas_actuators.ps1
```

## Uso rapido
```powershell
cd tools\atlas_actuators
node atlas_vision_capture.js --mode playwright --target http://127.0.0.1:8791/health --core-token "<TOKEN>"
node atlas_action_trigger.js --engine playwright --target http://127.0.0.1:8791 --actions "[{\"type\":\"wait\",\"ms\":500}]" --core-token "<TOKEN>"
node atlas_actuator_healthcheck.js
```
