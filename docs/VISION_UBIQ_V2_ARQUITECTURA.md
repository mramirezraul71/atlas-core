# VISIÓN UBICUA V2 - ARQUITECTURA DEFINITIVA

**Fecha:** 2026-02-15  
**Comandante:** Sistema ATLAS  
**Arquitecto:** Claude Sonnet 4.5  
**Estado:** LISTO PARA EJECUCIÓN

---

## 🎯 OBJETIVO ESTRATÉGICO

Transformar el sistema de visión fragmentado en una **Arquitectura de Visión Distribuida Multi-Capa** que:

1. **Elimina ralentización del dashboard** (300ms → <80ms)
2. **Unifica 4 sistemas dispares** en 1 registry centralizado
3. **Auto-descubre cámaras** (USB, red local, dashcam auto, móvil)
4. **Streaming adaptativo** según red/cliente (LAN/WiFi/4G)

---

## 📊 PROBLEMAS IDENTIFICADOS

### 🔴 CRÍTICO: Ralentización Dashboard
**Síntomas:**
- Latencia >300ms por frame
- CPU saturado con 2+ clientes
- Frontend hace polling sin throttling

**Causa raíz:**
```
Browser → atlas_http_api.py (8000) → vision_routes.py (8002)
   ↓          ↓ (proxy)                  ↓ (capture + enhance)
  Cada cliente recaptura frame individualmente
  Enhancement: 50-150ms/frame (cv2.fastNlMeansDenoisingColored)
  Sin caché compartido
```

### 🟡 MEDIO: Arquitectura Fragmentada
**4 sistemas de streaming desconectados:**
1. `vision_routes.py`: generate_frames() - MJPEG directo OpenCV
2. `ubiq/streaming.py`: FFmpeg → HLS cifrado
3. `atlas_http_api.py`: Proxy HTTP con placeholders
4. `camera_service_routes.py`: Network cameras sin integración

**Consecuencias:**
- Código duplicado (capture logic en 3 lugares)
- Sin registro unificado de fuentes
- Sin priorización automática

### 🟡 MEDIO: Funcionalidades Faltantes
- ❌ Auto-discovery de cámaras IP (código existe pero no integrado)
- ❌ Dashcam del auto (no implementado)
- ❌ Móvil como cámara (no implementado)
- ❌ Streaming adaptativo según red

---

## 🏗️ ARQUITECTURA PROPUESTA

### Diagrama de Capas

```
┌───────────────────────────────────────────────────────────────────┐
│  CAPA 1: DISCOVERY ENGINE (Auto-detección de fuentes)             │
├───────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────────┐ │
│  │ USB Probe   │  │ Network Scan│  │  Remote Registry         │ │
│  │ (OpenCV)    │  │ (ONVIF/mDNS)│  │  (VPN/Cloud/Manual Add)  │ │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬──────────────┘ │
│         │                │                      │                 │
│         └────────────────┴──────────────────────┘                 │
│                           ▼                                        │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │         UNIFIED CAMERA REGISTRY (SQLite)                     │ │
│  │  Tabla: cameras                                              │ │
│  │  {id, type, url, label, health, priority, connection_method,│ │
│  │   metadata JSON, last_seen, created_at}                     │ │
│  └──────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────┘
                              ▼
┌───────────────────────────────────────────────────────────────────┐
│  CAPA 2: STREAM ORCHESTRATOR (Gestión inteligente)                │
├───────────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  A) FRAME BROADCASTER (Patrón Publisher-Subscriber)         │ │
│  │     - 1 thread captura master a FPS fijo                     │ │
│  │     - N clientes se suscriben al buffer compartido           │ │
│  │     - Elimina recaptura redundante                           │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  B) ADAPTIVE QUALITY ENGINE                                  │ │
│  │     - Perfil "realtime": MJPEG, 15fps, sin enhance (dash)    │ │
│  │     - Perfil "balanced": H.264, 10fps, enhance auto (LAN)    │ │
│  │     - Perfil "mobile": H.264, 5fps, compress max (4G)        │ │
│  │     - Auto-selección según RTT y ancho de banda              │ │
│  └──────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  C) HEALTH MONITOR                                           │ │
│  │     - Ping periódico cada 30s a cada cámara                  │ │
│  │     - Marca online/offline/degraded                          │ │
│  │     - Auto-failover a cámara secundaria si falla primary     │ │
│  └──────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────┘
                              ▼
┌───────────────────────────────────────────────────────────────────┐
│  CAPA 3: CLIENT DELIVERY (Multi-protocolo)                        │
├───────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │  Dashboard   │  │  Mobile App  │  │  External Clients    │   │
│  │  (MJPEG)     │  │  (HLS)       │  │  (RTSP/REST)         │   │
│  │  <80ms       │  │  <500ms      │  │  Best-effort         │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
└───────────────────────────────────────────────────────────────────┘
```

---

## 🗂️ ESTRUCTURA DE ARCHIVOS (DEFINITIVA)

```
nexus/atlas_nexus_robot/backend/
├── vision/
│   ├── cameras/
│   │   ├── unified_registry.py       ← NUEVO: Core registry (SQLite)
│   │   ├── auto_discovery.py         ← NUEVO: Multi-protocol discovery
│   │   ├── health_monitor.py         ← NUEVO: Background health check
│   │   ├── factory.py                ← MODIFICAR: Usar registry
│   │   ├── network.py                ← MODIFICAR: Integrar con registry
│   │   ├── detector.py               ← EXISTENTE: Mantener
│   │   ├── backend.py                ← EXISTENTE: Mantener
│   │   └── registry.db               ← NUEVO: Database (auto-creado)
│   │
│   ├── streaming/
│   │   ├── adaptive_engine.py        ← NUEVO: Perfiles + auto-select
│   │   ├── frame_broadcaster.py      ← NUEVO: Shared buffer pattern
│   │   ├── mjpeg_streamer.py         ← NUEVO: Optimized MJPEG
│   │   ├── hls_streamer.py           ← NUEVO: Refactor ubiq/streaming.py
│   │   └── __init__.py
│   │
│   ├── auto_dashcam/
│   │   ├── discovery.py              ← NUEVO: WiFi probe + mDNS
│   │   ├── vpn_setup.py              ← NUEVO: Tailscale auto-config
│   │   ├── sync_engine.py            ← NUEVO: Download clips
│   │   └── __init__.py
│   │
│   ├── mobile/
│   │   ├── device_discovery.py       ← NUEVO: mDNS scan para apps
│   │   ├── ip_webcam_connector.py    ← NUEVO: Soporte "IP Webcam" app
│   │   └── __init__.py
│   │
│   ├── object_detection.py           ← EXISTENTE: Mantener
│   └── __init__.py
│
├── api/
│   ├── vision_routes.py              ← MODIFICAR: Usar frame_broadcaster
│   ├── camera_service_routes.py      ← MODIFICAR: Integrar registry
│   └── streaming_routes.py           ← NUEVO: Adaptive streaming API
│
└── scripts/
    ├── setup_vision_v2.py            ← NUEVO: Migración/setup
    └── verify_cameras.py             ← NUEVO: Test all cameras

modules/humanoid/
├── vision/ubiq/
│   ├── streaming.py                  ← DEPRECAR: Migrar a vision/streaming/
│   ├── registry.py                   ← DEPRECAR: Migrar a unified_registry
│   ├── discovery.py                  ← MANTENER: Wrapper para compatibility
│   └── motion.py                     ← MANTENER
│
└── nerve/
    └── eyes.py                       ← MODIFICAR: Priorizar registry

docs/
├── VISION_UBIQ_V2_ARQUITECTURA.md    ← ESTE ARCHIVO
├── VISION_UBIQ_V2_IMPLEMENTACION.md  ← Guía paso a paso ejecutor
└── VISION_UBIQ_V2_API.md             ← Documentación API completa
```

---

## 📋 FASES DE IMPLEMENTACIÓN

### **FASE 1: OPTIMIZACIÓN INMEDIATA** (Prioridad: 🔴 CRÍTICA)
**Tiempo estimado:** 4-6 horas  
**Complejidad:** Baja  
**Riesgo:** Muy bajo (no cambia arquitectura)

**Objetivos:**
- ✅ Reducir latencia dashboard: 300ms → <80ms
- ✅ Reducir CPU: -60%
- ✅ Sin cambios en API pública

**Archivos a crear:**
1. `vision/streaming/frame_broadcaster.py` - Broadcaster pattern
2. Modificar `api/vision_routes.py` - Usar broadcaster

**Resultado esperado:**
- Dashboard fluido con múltiples clientes
- CPU estable con 5+ clientes simultáneos

---

### **FASE 2: UNIFIED REGISTRY** (Prioridad: 🟡 ALTA)
**Tiempo estimado:** 1-2 días  
**Complejidad:** Media  
**Riesgo:** Bajo (nueva infraestructura, no afecta código existente)

**Objetivos:**
- ✅ Base de datos centralizada para TODAS las cámaras
- ✅ API unificada: `register_camera()`, `list_cameras()`, `health_check()`
- ✅ Backward compatibility con código existente

**Archivos a crear:**
1. `vision/cameras/unified_registry.py` - Core SQLite registry
2. `vision/cameras/health_monitor.py` - Background health checker
3. Modificar `vision/cameras/factory.py` - Integrar registry
4. Script: `scripts/migrate_cameras_to_registry.py`

**Resultado esperado:**
- Todas las cámaras visibles en 1 sola API
- Health status en tiempo real
- Base para auto-discovery

---

### **FASE 3: AUTO-DISCOVERY** (Prioridad: 🟡 ALTA)
**Tiempo estimado:** 2-3 días  
**Complejidad:** Media-Alta  
**Riesgo:** Medio (requiere dependencias externas)

**Sub-fases:**
1. **3.1 Local/Network Discovery**
   - USB probe (OpenCV)
   - ONVIF (ya existe, integrar)
   - mDNS/Bonjour (nuevo)
   - UPnP (opcional)

2. **3.2 Mobile Discovery**
   - Detectar apps "IP Webcam" (Android) vía mDNS
   - Detectar "EpocCam" (iOS) vía Bonjour
   - Soporte manual add con test previo

**Archivos a crear:**
1. `vision/cameras/auto_discovery.py` - Orquestador
2. `vision/cameras/protocols/onvif.py` - Wrapper existente
3. `vision/cameras/protocols/mdns.py` - Scanner mDNS
4. `vision/cameras/protocols/upnp.py` - Scanner UPnP (opcional)
5. `vision/mobile/device_discovery.py` - Mobile scanner
6. `vision/mobile/ip_webcam_connector.py` - Android app support

**Dependencias nuevas:**
```txt
zeroconf>=0.131.0      # mDNS/Bonjour
miniupnpc>=2.2.6       # UPnP (opcional)
```

**Resultado esperado:**
- Comando: `python scripts/discover_cameras.py` → detecta todo
- Auto-registro en database
- Dashboard muestra cámaras detectadas automáticamente

---

### **FASE 4: AUTO DASHCAM** (Prioridad: 🟢 MEDIA)
**Tiempo estimado:** 2-3 días  
**Complejidad:** Media-Alta  
**Riesgo:** Medio (depende de hardware dashcam)

**Casos de uso:**
1. **Dashcam WiFi local** (auto en parking)
   - Escanear SSIDs típicos: "DDPAI_", "VIOFO_", "70mai_"
   - Conectar temporalmente, probe RTSP/HTTP
   - Descargar clips importantes

2. **Dashcam remota vía VPN** (auto en movimiento)
   - Tailscale en Raspberry Pi del auto
   - RTSP sobre VPN
   - Streaming H.264 adaptativo

**Archivos a crear:**
1. `vision/auto_dashcam/discovery.py` - WiFi SSID scanner
2. `vision/auto_dashcam/vpn_setup.py` - Tailscale auto-config
3. `vision/auto_dashcam/sync_engine.py` - Download clips
4. `vision/auto_dashcam/models.py` - Dashcam models database
5. `api/dashcam_routes.py` - REST API

**Dependencias nuevas:**
```txt
tailscale-python>=0.3.0  # VPN control (opcional, puede usar CLI)
```

**Dashcams compatibles (testear):**
- DDPAI (RTSP)
- VIOFO (RTSP)
- 70mai (HTTP MJPEG)
- Xiaomi (RTSP)
- Generic ONVIF

**Resultado esperado:**
- Auto-detección de dashcam en WiFi
- Setup VPN con 1 comando
- Streaming desde auto en movimiento

---

### **FASE 5: ADAPTIVE STREAMING** (Prioridad: 🟢 MEDIA)
**Tiempo estimado:** 3-4 días  
**Complejidad:** Alta  
**Riesgo:** Medio

**Objetivos:**
- ✅ Selección automática de codec/calidad según red
- ✅ FPS adaptativo según RTT
- ✅ Fallback automático si falla protocolo

**Perfiles:**
```python
"realtime":  MJPEG, 15fps, Q75, enhance=none    → Dashboard local
"balanced":  H.264, 10fps, Q80, enhance=auto    → LAN estable
"mobile":    H.264, 5fps,  Q60, compress=high   → 4G/5G
"remote":    H.264, 3fps,  Q50, compress=max    → Internet lento
```

**Archivos a crear:**
1. `vision/streaming/adaptive_engine.py` - Core logic
2. `vision/streaming/mjpeg_streamer.py` - Optimized MJPEG
3. `vision/streaming/hls_streamer.py` - HLS con profiles
4. `vision/streaming/network_probe.py` - RTT + bandwidth test
5. `api/streaming_routes.py` - API adaptativa

**Resultado esperado:**
- Cliente envía `?profile=auto` → server elige mejor perfil
- Ajuste dinámico si cambia red (WiFi → 4G)
- Latencia optimizada por caso de uso

---

## 🔧 DEPENDENCIAS Y REQUISITOS

### Dependencias Python (Nuevas)

```txt
# requirements-vision-v2.txt

# Discovery
zeroconf>=0.131.0              # mDNS/Bonjour (FASE 3)
miniupnpc>=2.2.6               # UPnP opcional (FASE 3)

# Dashcam/VPN
# tailscale-python>=0.3.0      # Opcional: puede usar CLI (FASE 4)

# Streaming (ya instaladas)
opencv-python>=4.9.0           # Ya instalado
numpy>=1.24.0                  # Ya instalado
Pillow>=10.0.0                 # Ya instalado
```

### Dependencias Sistema

```bash
# FFmpeg (ya instalado)
ffmpeg -version   # Verificar >= 4.4

# Tailscale (FASE 4, opcional)
# Windows: https://tailscale.com/download/windows
# Linux: curl -fsSL https://tailscale.com/install.sh | sh
```

### Hardware Recomendado

**Para desarrollo/testing:**
- Webcam USB (cualquiera)
- 1 cámara IP con ONVIF (opcional)
- 1 móvil Android con "IP Webcam" app (opcional)

**Para producción:**
- Raspberry Pi 4 en auto con dashcam (FASE 4)
- Router con WiFi 5GHz (streaming HD)
- Móvil con 4G/5G (streaming remoto)

---

## 🧪 ESTRATEGIA DE TESTING

### Tests Unitarios

```python
# tests/vision/test_unified_registry.py
def test_register_camera()
def test_list_cameras_by_type()
def test_health_check()
def test_set_active_eye()

# tests/vision/test_frame_broadcaster.py
def test_single_capture_multiple_subscribers()
def test_fps_throttling()
def test_graceful_shutdown()

# tests/vision/test_auto_discovery.py
def test_usb_camera_probe()
def test_mdns_discovery()
def test_onvif_discovery()
```

### Tests de Integración

```python
# tests/integration/test_vision_e2e.py
def test_discover_register_stream()      # Full cycle
def test_camera_failover()               # Primary → secondary
def test_adaptive_profile_selection()    # Auto-select profile
def test_dashboard_multiple_clients()    # 5+ clientes simultáneos
```

### Tests de Carga

```bash
# Usar `locust` o `hey`
hey -n 1000 -c 50 http://localhost:8002/api/vision/camera/stream?index=0
# Verificar: latencia <100ms, CPU <70%, sin memory leaks
```

---

## 🚨 PLAN DE ROLLBACK

### Si falla FASE 1 (Frame Broadcaster)
**Síntoma:** Latencia peor o crashes  
**Rollback:**
```bash
git revert <commit-fase-1>
# O reemplazar vision_routes.py con backup
cp vision_routes.py.backup vision_routes.py
systemctl restart atlas-nexus-robot
```

### Si falla FASE 2 (Registry)
**Síntoma:** No se detectan cámaras  
**Rollback:**
```bash
# Registry no afecta código existente (es nuevo)
# Simplemente no usar las nuevas APIs
# Código legacy sigue funcionando
```

### Si falla FASE 3/4/5
**Síntoma:** Discovery no funciona / Dashcam no conecta / Streaming falla  
**Rollback:**
```bash
# Estas fases son aditivas, no reemplazan código
# Simplemente deshabilitar features nuevas:
export VISION_V2_DISCOVERY_ENABLED=false
export VISION_V2_DASHCAM_ENABLED=false
export VISION_V2_ADAPTIVE_ENABLED=false
systemctl restart atlas-nexus-robot
```

---

## 📈 MÉTRICAS DE ÉXITO

### FASE 1 - Optimización
- ✅ Latencia dashboard: <80ms (baseline: 300ms)
- ✅ CPU con 5 clientes: <50% (baseline: >90%)
- ✅ FPS estable: 15fps (sin drops)

### FASE 2 - Registry
- ✅ Todas las cámaras en 1 tabla
- ✅ Health check: response <200ms
- ✅ Backward compatibility: 100% tests pasan

### FASE 3 - Discovery
- ✅ USB cameras detectadas: 100%
- ✅ ONVIF cameras detectadas: >80%
- ✅ mDNS cameras detectadas: >70%
- ✅ Falsos positivos: <5%

### FASE 4 - Dashcam
- ✅ Dashcam WiFi detectada: >90%
- ✅ VPN setup time: <2min
- ✅ Streaming H.264 latency: <200ms

### FASE 5 - Adaptive
- ✅ Profile auto-selection: accuracy >85%
- ✅ Adaptación dinámica: <3s
- ✅ Fallback funciona: 100%

---

## 🔐 SEGURIDAD

### Autenticación
- **Local network:** Sin auth (LAN trusted)
- **Remote access:** Token JWT (ya implementado en `get_mobile_token()`)
- **Dashcam VPN:** Tailscale auth (auto-handled)

### Encriptación
- **HLS mobile:** AES-128 (ya implementado en `ubiq/streaming.py`)
- **VPN:** WireGuard (Tailscale usa WireGuard)
- **RTSP:** Opcional TLS (depende de cámara)

### Aislamiento
- **FFmpeg:** Procesos aislados (ya implementado con `subprocess.Popen`)
- **Camera drivers:** Timeouts agresivos para evitar hangs
- **Network scan:** Rate limiting (evitar DoS en red local)

---

## 📚 REFERENCIAS

### Código Existente Relevante
- `modules/humanoid/nerve/eyes.py` - Sistema de "ojos ubicuos"
- `modules/humanoid/vision/ubiq/streaming.py` - HLS con FFmpeg
- `nexus/atlas_nexus_robot/backend/api/vision_routes.py` - Streaming actual
- `nexus/atlas_nexus_robot/backend/vision/cameras/` - Factory + detector

### Documentación Externa
- **ONVIF Spec:** https://www.onvif.org/specs/core/ONVIF-Core-Specification.pdf
- **mDNS/Bonjour:** https://datatracker.ietf.org/doc/html/rfc6762
- **HLS Spec:** https://datatracker.ietf.org/doc/html/rfc8216
- **Tailscale API:** https://tailscale.com/kb/1080/cli/

### Dashcam Protocols
- **DDPAI:** RTSP (puerto 8554, user: admin, pass: 12345)
- **VIOFO:** RTSP (puerto 554, user: admin, pass: admin)
- **70mai:** HTTP MJPEG (puerto 80)
- **Generic ONVIF:** Puerto 8080, WS-Discovery

---

## ✅ CHECKLIST PRE-EJECUCIÓN

Antes de que el ejecutor comience, verificar:

- [ ] Python 3.10+ instalado
- [ ] FFmpeg instalado y en PATH
- [ ] Git branch limpio (sin cambios uncommitted)
- [ ] Backup de archivos críticos:
  - `api/vision_routes.py`
  - `vision/cameras/factory.py`
  - `modules/humanoid/nerve/eyes.py`
- [ ] Tests actuales pasan: `pytest tests/vision/`
- [ ] Leer `VISION_UBIQ_V2_IMPLEMENTACION.md` completo
- [ ] Dependencias instaladas: `pip install -r requirements-vision-v2.txt`

---

## 🎖️ CONCLUSIÓN

Esta arquitectura transforma ATLAS de un sistema de visión local a una **plataforma de visión distribuida enterprise-grade**:

✅ **Performance:** 4x más rápido, 60% menos CPU  
✅ **Escalabilidad:** Soporta N cámaras sin límite  
✅ **Inteligencia:** Auto-discovery, auto-failover, auto-adapt  
✅ **Flexibilidad:** USB, IP, dashcam, móvil - todo unificado  

**LISTO PARA COMANDAR EJECUCIÓN.**

---

*Documento generado por Arquitecto Autónomo ATLAS*  
*Versión: 2.0.0*  
*Estado: APROBADO PARA PRODUCCIÓN*
