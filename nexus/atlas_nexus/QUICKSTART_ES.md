# ⚡ ATLAS NEXUS - Guía de Inicio Rápido

## 🎯 ¿Qué es esto?

**ATLAS NEXUS** es la versión profesional y mejorada de tu ATLAS_PUSH. Incluye:

✅ **Múltiples IA integradas** (Ollama + DeepSeek + Claude + GPT)
✅ **Autonomía real** con planificación multi-paso
✅ **50+ herramientas profesionales**
✅ **API REST completa** para control desde móvil
✅ **WebSocket en tiempo real**
✅ **Auto-recuperación de errores**
✅ **Sistema modular y escalable**

---

## 🚀 Instalación Rápida (Windows)

### Paso 1: Copiar ATLAS NEXUS a tu PC

```powershell
# Copia la carpeta ATLAS_NEXUS a:
C:\ATLAS_NEXUS\

# O donde prefieras
```

### Paso 2: Instalar

```powershell
# Abre PowerShell en la carpeta ATLAS_NEXUS
cd C:\ATLAS_NEXUS

# Ejecuta el instalador
PowerShell -ExecutionPolicy Bypass -File install.ps1
```

Esto va a:
- ✓ Crear entorno virtual Python
- ✓ Instalar todas las dependencias
- ✓ Crear carpetas necesarias
- ✓ Generar archivo de configuración

### Paso 3: Configurar

```powershell
# Edita el archivo de configuración
notepad config\.env
```

**Agrega tus API keys:**
```env
# Ollama (ya tienes esto instalado)
OLLAMA_BASE_URL=http://localhost:11434

# OpenAI (opcional)
OPENAI_API_KEY=sk-...

# Claude (opcional)
ANTHROPIC_API_KEY=sk-ant-...

# Telegram (ya lo tienes)
TELEGRAM_BOT_TOKEN=tu-token-aqui
TELEGRAM_OWNER_ID=tu-id-aqui
```

### Paso 4: Iniciar

```powershell
# Modo API (recomendado para móvil)
python nexus.py --mode api

# O usa el script de inicio
PowerShell -ExecutionPolicy Bypass -File start.ps1
```

✅ **Listo!** ATLAS NEXUS está corriendo en `http://localhost:8000`

---

## 📱 Acceso desde Móvil

### Opción 1: API REST

Tu móvil puede conectarse a: `http://tu-ip:8000`

**Endpoints principales:**
```
POST /goal         → Dar una tarea autónoma
GET  /status       → Ver estado del sistema
POST /think        → Consulta directa a la IA
GET  /tools        → Ver herramientas disponibles
WS   /ws           → WebSocket para actualizaciones en tiempo real
```

### Opción 2: Telegram (Ya funcional)

Tu bot de Telegram sigue funcionando igual, pero ahora con más poder.

---

## 🔥 Ejemplos de Uso

### Ejemplo 1: Desde Móvil (API)

```javascript
// Dar una tarea compleja
fetch('http://tu-pc:8000/goal', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({
    goal: 'Busca las últimas noticias de IA y crea un resumen'
  })
})
```

### Ejemplo 2: Desde Python

```python
from nexus import AtlasNexus

nexus = AtlasNexus()

# Tarea autónoma
plan = await nexus.achieve_goal(
    "Analiza este CSV y crea un reporte en PDF"
)

print(f"Completado: {plan.status}")
```

### Ejemplo 3: Modo Interactivo

```powershell
python nexus.py --mode interactive
```

Luego escribe comandos:
```
You: crea un script de Python para analizar ventas
Atlas: [ejecuta autónomamente la tarea]

You: status
Atlas: [muestra estado del sistema]
```

---

## 🛠️ Migrar tu ATLAS_PUSH Actual

Si quieres mantener tus datos del ATLAS_PUSH anterior:

```powershell
# Ejecuta el script de migración
python migrate.py \
  --old "C:\Users\r6957\OneDrive\Desktop\ATLAS_PUSH" \
  --new "C:\ATLAS_NEXUS"
```

Esto copiará:
- ✓ Configuración de Telegram
- ✓ Memoria y logs
- ✓ Snapshots
- ✓ Módulos personalizados

---

## 🎨 Características Principales

### 1. Inteligencia Híbrida

```python
# ATLAS escoge automáticamente el mejor modelo:
# - DeepSeek Coder → para código
# - DeepSeek R1 → para razonamiento
# - Claude → para creatividad
# - Llama → para tareas rápidas
```

### 2. Autonomía Real

```python
# Solo dale un objetivo:
"Crea una presentación sobre IA"

# ATLAS planifica y ejecuta:
# 1. Busca información
# 2. Procesa y analiza
# 3. Genera presentación
# 4. Te notifica cuando termina
```

### 3. Herramientas Profesionales

- **Web**: Búsqueda, scraping, navegación
- **Archivos**: Lectura, escritura, PDF, Excel
- **Sistema**: Comandos, monitoreo
- **Comunicación**: Telegram, email, SMS
- **Datos**: Análisis, visualización

### 4. Control Total desde Móvil

```
Tu App Móvil
     ↓
   REST API
     ↓
 ATLAS NEXUS
     ↓
   Resultado
```

---

## 📊 Documentación API

Accede a la documentación interactiva:
```
http://localhost:8000/docs
```

Ahí puedes:
- ✓ Ver todos los endpoints
- ✓ Probar la API directamente
- ✓ Ver ejemplos de código
- ✓ Generar clientes para tu app móvil

---

## 🔧 Configuración Avanzada

### Ollama Models

```bash
# Instala los modelos recomendados
ollama pull deepseek-coder:6.7b
ollama pull deepseek-r1:latest
ollama pull llama3.2:latest
ollama pull nomic-embed-text  # para embeddings

# Ver modelos instalados
ollama list
```

### Cambiar Puerto API

```env
# En config\.env
ATLAS_API_PORT=8001
```

### Nivel de Autonomía

```env
# En config\.env
ATLAS_AUTONOMY=high  # low, medium, high, full
```

---

## 🐛 Solución de Problemas

### Problema: "Ollama not found"
```bash
# Instala Ollama desde https://ollama.ai
# Luego descarga los modelos (ver arriba)
```

### Problema: "Port already in use"
```powershell
# Cambia el puerto en config\.env
ATLAS_API_PORT=8001
```

### Problema: "Import errors"
```powershell
# Reinstala dependencias
pip install -r requirements.txt --upgrade
```

### Problema: "Cannot connect from mobile"
```powershell
# 1. Verifica que tu PC y móvil estén en la misma red
# 2. Obtén la IP de tu PC: ipconfig
# 3. Usa esa IP en tu app móvil: http://192.168.x.x:8000
# 4. Si hay firewall, permite el puerto 8000
```

---

## 📈 Próximos Pasos

1. **Experimenta con el modo interactivo**
   ```bash
   python nexus.py --mode interactive
   ```

2. **Prueba la API desde tu navegador**
   ```
   http://localhost:8000/docs
   ```

3. **Conecta desde tu móvil**
   - Encuentra tu IP local
   - Crea requests HTTP desde tu app

4. **Explora las herramientas disponibles**
   ```bash
   GET http://localhost:8000/tools
   ```

5. **Crea herramientas personalizadas**
   - Mira `tools/tools_registry.py`
   - Agrega tus propias herramientas

---

## 💡 Consejos

- 📝 **Logs**: Revisa `logs/nexus.log` para debug
- 🔄 **Snapshots**: El sistema hace backups automáticos
- 🎯 **Tareas complejas**: Usa `/goal` para autonomía
- ⚡ **Tareas simples**: Usa `/think` para respuestas rápidas
- 📱 **Móvil**: WebSocket (`/ws`) para updates en tiempo real

---

## 🎯 Diferencias vs ATLAS_PUSH

| Característica | ATLAS_PUSH | ATLAS NEXUS |
|---|---|---|
| Modelos IA | OpenAI | Ollama + DeepSeek + Claude + GPT |
| Autonomía | Básica | Avanzada con planificación |
| Herramientas | ~10 | 50+ |
| API | Básica | REST + WebSocket completo |
| Móvil | Telegram | API nativa + Telegram |
| Recuperación | Manual | Automática |
| Escalabilidad | Limitada | Profesional |

---

## 📞 Soporte

- 📖 **Docs API**: http://localhost:8000/docs
- 📝 **Logs**: `logs/nexus.log`
- 🐛 **Issues**: Crea un issue en el repo

---

**¡Disfruta tu nuevo ATLAS NEXUS!** 🚀

*Sistema construido con autonomía total para ser poderoso, profesional y escalable.*
