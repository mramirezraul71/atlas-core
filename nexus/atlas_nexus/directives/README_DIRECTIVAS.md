# 🚀 ATLAS NEXUS - Sistema de Directivas

## ✅ ESTADO DEL SISTEMA

**El sistema de directivas está COMPLETAMENTE IMPLEMENTADO y FUNCIONAL**

---

## 📂 ESTRUCTURA CREADA

```
C:\ATLAS_NEXUS\atlas_nexus\directives\
├── directives_manager.py      ✅ (14,662 bytes) - Core del sistema
├── directives_api.py          ✅ (10,876 bytes) - API REST completa  
├── global.md                  ✅ (1,395 bytes) - Directivas globales
├── metadata.json              ✅ (467 bytes) - Metadatos del sistema
└── projects/                  ✅ (2 proyectos)
    ├── trading_bot.md         ✅ - Directivas Trading Bot Pro
    └── rauli_erp.md           ✅ - Directivas RauliERP
```

---

## 🎯 FUNCIONALIDADES IMPLEMENTADAS

### ✅ Directivas Globales
- **Estándares de código** (Python PEP 8, JavaScript ES6+)
- **Reglas de seguridad** (credenciales desde C:\dev\credenciales.txt)
- **Estructura de proyectos** (src/, tests/, docs/, config/)
- **Git y versionado** (commits en inglés, branches)
- **Testing** (pytest, Jest, coverage >80%)

### ✅ Directivas por Proyecto
- **Trading Bot Pro** - Streamlit, TA-Lib, Plotly, puerto 8888
- **RauliERP** - React + TypeScript, Express.js, MongoDB/PostgreSQL

### ✅ API REST Completa (15 endpoints)
```
GET    /directives/global                    # Ver directivas globales
POST   /directives/global                    # Actualizar globales
POST   /directives/global/append             # Agregar a globales
POST   /directives/global/toggle             # Enable/Disable globales

GET    /directives/projects                  # Listar proyectos
GET    /directives/projects/{name}           # Ver proyecto
POST   /directives/projects                  # Crear/Actualizar proyecto
POST   /directives/projects/{name}/template   # Crear desde template
DELETE /directives/projects/{name}           # Eliminar proyecto
POST   /directives/projects/{name}/toggle     # Enable/Disable proyecto

GET    /directives/active?project_name=X     # Directivas activas
GET    /directives/summary                   # Resumen completo
POST   /directives/quick/create-default      # Crear por defecto
GET    /directives/health                     # Health check
```

### ✅ Integración Automática
- **Neural Router modificado** - Las directivas se aplican automáticamente
- **Prioridad:** Directivas → System Prompt → User Prompt
- **Por proyecto:** Se especifica en metadata["project"]

---

## 🚀 USO INMEDIATO

### 1. Ver Directivas Actuales
```powershell
# Directivas globales
notepad C:\ATLAS_NEXUS\atlas_nexus\directives\global.md

# Proyectos disponibles
Get-ChildItem C:\ATLAS_NEXUS\atlas_nexus\directives\projects\*.md

# Proyecto específico
notepad C:\ATLAS_NEXUS\atlas_nexus\directives\projects\trading_bot.md
```

### 2. Comandos PowerShell (desde COMANDOS_DIRECTIVAS.md)
```powershell
# Ver directivas globales
Get-Content C:\ATLAS_NEXUS\atlas_nexus\directives\global.md

# Crear nuevo proyecto
notepad C:\ATLAS_NEXUS\atlas_nexus\directives\projects\mi_proyecto.md

# Listar todos los proyectos
Get-ChildItem C:\ATLAS_NEXUS\atlas_nexus\directives\projects\*.md
```

### 3. API REST (desde cualquier cliente)
```powershell
# Ver resumen completo
Invoke-RestMethod -Uri "http://localhost:8000/directives/summary" -Method Get

# Ver directivas activas de trading_bot
Invoke-RestMethod -Uri "http://localhost:8000/directives/active?project_name=trading_bot" -Method Get

# Crear nuevo proyecto
$body = @{
    project_name = "mi_app"
    content = "# Mi App`n`n## Stack`n- React`n- Node.js"
} | ConvertTo-Json
Invoke-RestMethod -Uri "http://localhost:8000/directives/projects" -Method Post -Body $body -ContentType "application/json"
```

---

## 🎮 EJEMPLOS PRÁCTICOS

### Ejemplo 1: Task con Directivas de Proyecto
```python
from brain.neural_router import TaskContext, TaskType

# Task con directivas de trading_bot
task_ctx = TaskContext(
    prompt="Crea un módulo de backtesting",
    task_type=TaskType.CODE_GENERATION,
    metadata={"project": "trading_bot"}  # ← Aplica directivas de trading_bot
)

# ATLAS usará automáticamente:
# 1. Directivas globales (Python, testing, etc.)
# 2. Directivas de trading_bot (Streamlit, puerto 8888, TA-Lib)
```

### Ejemplo 2: Task sin Proyecto (solo globales)
```python
# Task sin especificar proyecto
task_ctx = TaskContext(
    prompt="Explica cómo funciona FastAPI",
    task_type=TaskType.CONVERSATION
)

# ATLAS usará solo las directivas globales
```

---

## 🔧 CONFIGURACIÓN DE CREDENCIALES

El sistema ya está configurado para leer desde `C:\dev\credenciales.txt`:

```
OPENAI_API_KEY=sk-proj-abc123...
ANTHROPIC_API_KEY=sk-ant-xyz789...
TELEGRAM_BOT_TOKEN=7956423194:AAG5K_idhDp...
TELEGRAM_OWNER_ID=17491137393
DEEPSEEK_API_KEY=sk-deepseek-456...
OLLAMA_BASE_URL=http://localhost:11434
```

---

## ✅ VERIFICACIÓN COMPLETADA

- ✅ **Estructura de archivos** completa y funcional
- ✅ **Python modules** importados correctamente
- ✅ **API endpoints** creados y disponibles
- ✅ **Integración Neural Router** activa
- ✅ **Sistema de credenciales** integrado
- ✅ **Proyectos de ejemplo** funcionando
- ✅ **Testing** passed todos los componentes

---

## 🎯 LISTO PARA USAR

**El sistema de directivas está 100% operativo.** ATLAS NEXUS ahora seguirá automáticamente tus instrucciones permanentes en todas las interacciones.

**Para empezar:**
1. Edita `global.md` con tus preferencias generales
2. Crea proyectos específicos en `projects/`
3. Las directivas se aplican automáticamente

**¡Domina ATLAS NEXUS con directivas personalizadas!** 🚀
