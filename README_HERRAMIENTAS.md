# Herramientas Integradas en Atlas

## Estado Actual de Herramientas

### ✅ Instaladas y Activas
- **Mem0** (Memoria): `mem0ai v0.1.34` - Para recordar detalles de clientes y hábitos
- **Composio** (Conectividad): `v0.9.2` - Para usar apps externas (Excel, Notion, Gmail)
- **Ollama** (Motor IA): `v0.16.2` - Para ejecutar el "pensamiento" de ATLAS de forma privada

### ⚠️ Instaladas pero requieren Docker Desktop activo
- **Home Assistant** (IoT/Hardware): Para control físico de ATLAS_NEXUS y sensores
- **Appsmith** (Interfaz): Para crear panel visual de ganancias de trading

## Scripts de Configuración

### 1. Verificación e Instalación
```bash
python scripts/install_missing_tools.py
```

### 2. Iniciar Servicios Docker (requiere Docker Desktop)
```powershell
.\scripts\docker_services_setup.ps1
```

### 3. Inicio Manual Home Assistant (alternativa sin Docker)
```bash
python scripts/start_homeassistant.py
```

### 4. Configuración de Integraciones
```bash
python scripts/setup_integrations.py
```

### 5. Verificación de Estado
```bash
python config/atlas_tools_config.py
```

## URLs de Acceso

- **Home Assistant**: http://localhost:8123
- **Appsmith**: http://localhost  
- **Ollama API**: http://localhost:11434

## Configuración de API Keys

Opcionalmente configura las siguientes variables de entorno:

```bash
# Mem0 (para memoria avanzada)
set MEM0_API_KEY=tu-api-key-aqui

# Composio (para integraciones externas)
set COMPOSIO_API_KEY=tu-api-key-aqui
```

## Uso en Atlas

### Mem0 - Memoria
```python
from mem0 import Memory
memory = Memory()
# Recordar detalles de clientes y hábitos
```

### Composio - Conectividad  
```python
from composio import App, Action
# Integrar con Excel, Notion, Gmail sin programar
```

### Ollama - Motor IA
```python
import ollama
# Ejecutar modelos localmente para pensamiento privado
```

### Home Assistant - IoT
```python
# Control físico de ATLAS_NEXUS y sensores via http://localhost:8123
```

### Appsmith - Interfaz
```python
# Crear paneles visuales via http://localhost
```

## Próximos Pasos

1. **Iniciar Docker Desktop** si no está corriendo
2. **Ejecutar script Docker** para Home Assistant y Appsmith
3. **Configurar API keys** opcionalmente para funcionalidad extendida
4. **Verificar estado** con el script de configuración

## Troubleshooting

- **Docker no responde**: Inicia Docker Desktop manualmente
- **Home Assistant no inicia**: Usa el script manual `start_homeassistant.py`
- **Puertos ocupados**: Libera puertos 80, 443, 8123, 11434
- **Permisos Windows**: Ejecutar PowerShell como Administrador
