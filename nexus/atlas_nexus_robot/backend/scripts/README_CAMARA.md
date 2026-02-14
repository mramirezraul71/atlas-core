# Asistente de conexión de cámara

## Script de diagnóstico

```bash
cd C:\ATLAS_NEXUS\atlas_nexus_robot\backend
python scripts/camera_connect_helper.py
```

El script:
- Verifica OpenCV
- Lista dispositivos PnP (Windows)
- Escanea índices OpenCV 0–7
- Prueba lectura de frames
- Guarda la configuración activa

## API: forzar conexión

```bash
curl -X POST http://localhost:8002/api/camera/connect
```

O desde el navegador (DevTools) o Postman. Fuerza re-detección y prueba la cámara.

## Si la cámara no aparece

1. **Insta360 Link / Link 2**: Conecta en USB 3.0; cierra Zoom/Teams que usen la cámara.
2. **Índice OpenCV**: Algunas cámaras usan índice 1 o 2 en vez de 0. El script las prueba todas.
3. **Config manual**: `config/active_camera.json` — define `index`, `resolution`, `model`.
4. **Reiniciar backend**: Tras cambiar la config, reinicia el backend Robot (puerto 8002).

## Endpoints útiles

| Endpoint | Descripción |
|----------|-------------|
| GET `/api/camera/service/status` | Estado, cámaras detectadas, stream URL |
| GET `/api/camera/detect` | Detecta cámaras disponibles |
| POST `/api/camera/connect` | Fuerza conexión y verifica |
| POST `/api/camera/configure` | Configura índice/resolución (JSON body) |
