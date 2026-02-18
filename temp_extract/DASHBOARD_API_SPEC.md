# 🔌 ATLAS DASHBOARD - API ENDPOINTS SPECIFICATION

## ENDPOINTS REQUERIDOS PARA EL DASHBOARD

El dashboard necesita los siguientes endpoints para funcionar con datos reales.

---

## 1. HEALTH & SERVICES

### GET `/api/health`
**Descripción:** Estado general del servicio

**Response:**
```json
{
  "status": "healthy",
  "uptime_seconds": 170580,
  "uptime_formatted": "47h 23m",
  "port": 8791,
  "load_percent": 23,
  "timestamp": "2025-02-15T14:23:45Z"
}
```

---

## 2. LEARNING SYSTEM

### GET `/api/learning/status`
**Descripción:** Estado de la lección actual

**Response:**
```json
{
  "current_lesson": {
    "lesson_id": "L012",
    "name": "Advanced Object Manipulation",
    "progress_percent": 68,
    "tasks_completed": 12,
    "tasks_total": 18,
    "start_time": "2025-02-15T08:00:00Z"
  },
  "tutor_score": 85,
  "uncertainty_rate": 0.23,
  "learning_speed": "medium-fast"
}
```

### GET `/api/learning/tutor`
**Descripción:** Estado del tutor IA

**Response:**
```json
{
  "status": "active",
  "model": "claude-opus-4-20250514",
  "lessons_completed": 23,
  "lessons_total": 45,
  "robot_level": "intermediate",
  "average_score": 85.3,
  "next_review_in_seconds": 9240
}
```

---

## 3. MEMORY SYSTEMS

### GET `/api/memory/stats`
**Descripción:** Estadísticas de todos los sistemas de memoria

**Response:**
```json
{
  "episodic": {
    "total_episodes": 1247,
    "episodes_today": 34,
    "database_path": "logs/episodic_memory.sqlite",
    "database_size_mb": 12.4
  },
  "semantic": {
    "total_embeddings": 1247,
    "total_concepts": 156,
    "index_type": "FAISS",
    "dimensions": 384
  },
  "knowledge_base": {
    "total_concepts": 89,
    "total_skills": 23,
    "total_rules": 34,
    "learned_concepts": 34,
    "initial_concepts": 55
  }
}
```

---

## 4. PERFORMANCE METRICS

### GET `/api/metrics/performance`
**Descripción:** Métricas de desempeño del robot

**Response:**
```json
{
  "success_rate": 87.3,
  "average_confidence": 0.78,
  "consolidations_total": 12,
  "uncertainty_episodes": 45,
  "times_asked_for_help": 23,
  "learning_speed": "medium-fast"
}
```

---

## 5. RECENT ACTIVITY

### GET `/api/recent-activity?limit=10`
**Descripción:** Últimas acciones del robot

**Query Params:**
- `limit` (opcional, default: 10): Número de actividades

**Response:**
```json
{
  "activities": [
    {
      "timestamp": "2025-02-15T14:23:00Z",
      "time_formatted": "14:23",
      "action": "Identified fragile object",
      "status": "success",
      "confidence": 0.92,
      "task_type": "object_identification"
    },
    {
      "timestamp": "2025-02-15T14:21:00Z",
      "time_formatted": "14:21",
      "action": "Consulted AI Tutor",
      "status": "success",
      "confidence": 0.85,
      "task_type": "learning"
    },
    {
      "timestamp": "2025-02-15T14:18:00Z",
      "time_formatted": "14:18",
      "action": "Pick and place task",
      "status": "success",
      "confidence": 0.88,
      "task_type": "manipulation"
    }
  ]
}
```

**Status values:** `"success"` | `"warning"` | `"error"`

---

## 6. VISION SYSTEM

### GET `/api/vision/status`
**Descripción:** Estado de componentes de visión

**Response:**
```json
{
  "components": [
    {
      "name": "YOLO Detection",
      "status": "active",
      "model": "yolov8n",
      "fps": 30
    },
    {
      "name": "Depth Estimation (MiDaS)",
      "status": "active",
      "model": "DPT-Hybrid",
      "latency_ms": 45
    },
    {
      "name": "Scene Understanding (LLaVA)",
      "status": "active",
      "model": "llava-1.6-7b"
    },
    {
      "name": "Multi-Camera Fusion",
      "status": "standby",
      "cameras_active": 0,
      "cameras_total": 4
    }
  ]
}
```

**Status values:** `"active"` | `"standby"` | `"offline"`

---

## 7. SYSTEM HEALTH

### GET `/api/system/health`
**Descripción:** Métricas de salud del sistema

**Response:**
```json
{
  "cpu_usage_percent": 45,
  "memory_usage_percent": 67,
  "gpu_usage_percent": 82,
  "storage_usage_percent": 34,
  "temperature_celsius": 65,
  "uptime_seconds": 170580
}
```

---

## 8. CONSOLIDATION STATUS

### GET `/api/learning/consolidation/status`
**Descripción:** Estado de consolidación de conocimiento

**Response:**
```json
{
  "last_consolidation": "2025-02-15T10:00:00Z",
  "hours_since_last": 4.5,
  "total_consolidations": 12,
  "patterns_found": 34,
  "concepts_created": 8,
  "rules_updated": 15
}
```

---

## IMPLEMENTACIÓN EN ATLAS

### Agregar a `atlas_adapter/atlas_http_api.py`

```python
from datetime import datetime
from modules.humanoid.memory_engine.semantic_memory import semantic_memory
from brain.memory.episodic_memory import episodic_memory
from brain.knowledge.initial_knowledge import knowledge_base
from brain.learning.continual_learning_loop import learning_loop

# ═══ DASHBOARD ENDPOINTS ═══

@app.get("/api/health")
async def get_health():
    """Health status para dashboard"""
    import psutil
    import time
    
    uptime = time.time() - app.state.start_time
    
    return {
        "status": "healthy",
        "uptime_seconds": int(uptime),
        "uptime_formatted": f"{int(uptime//3600)}h {int((uptime%3600)//60)}m",
        "port": 8791,
        "load_percent": psutil.cpu_percent(interval=1),
        "timestamp": datetime.now().isoformat()
    }

@app.get("/api/learning/status")
async def get_learning_status():
    """Estado de aprendizaje para dashboard"""
    if not learning_loop.current_lesson:
        return {"current_lesson": None}
    
    lesson = learning_loop.current_lesson
    
    return {
        "current_lesson": {
            "lesson_id": lesson.get('lesson_id'),
            "name": lesson.get('name'),
            "progress_percent": learning_loop.calculate_progress(),
            "tasks_completed": len(learning_loop.daily_report['tasks_completed']),
            "tasks_total": len(lesson.get('tasks', [])),
            "start_time": learning_loop.lesson_start_time.isoformat()
        },
        "tutor_score": learning_loop.daily_report.get('last_score', 0),
        "uncertainty_rate": learning_loop.daily_report.get('uncertainty_episodes', 0) / max(learning_loop.experience_counter, 1),
        "learning_speed": "medium-fast"
    }

@app.get("/api/memory/stats")
async def get_memory_stats():
    """Estadísticas de memoria para dashboard"""
    episodic_stats = episodic_memory.get_statistics()
    semantic_stats = semantic_memory.get_statistics()
    kb_stats = knowledge_base.get_statistics()
    
    return {
        "episodic": {
            "total_episodes": episodic_stats['total_episodes'],
            "episodes_today": episodic_stats.get('today_episodes', 0),
            "database_size_mb": episodic_stats['database_size_mb']
        },
        "semantic": {
            "total_embeddings": semantic_stats['total_experiences'],
            "total_concepts": semantic_stats.get('unique_tags', 0),
            "index_type": "FAISS",
            "dimensions": 384
        },
        "knowledge_base": {
            "total_concepts": kb_stats['total_concepts'],
            "total_skills": kb_stats['total_skills'],
            "total_rules": kb_stats['total_rules'],
            "learned_concepts": kb_stats['learned_concepts'],
            "initial_concepts": kb_stats['initial_concepts']
        }
    }

@app.get("/api/recent-activity")
async def get_recent_activity(limit: int = 10):
    """Actividad reciente para dashboard"""
    episodes = episodic_memory.get_recent_episodes(limit=limit)
    
    activities = []
    for ep in episodes:
        activities.append({
            "timestamp": ep['created_at'],
            "time_formatted": datetime.fromisoformat(ep['created_at']).strftime("%H:%M"),
            "action": ep['situation'],
            "status": "success" if ep['success'] else "error",
            "confidence": 1.0 - ep['uncertainty_score'],
            "task_type": ep['task_type']
        })
    
    return {"activities": activities}

# Agregar en startup event:
@app.on_event("startup")
async def startup_event():
    app.state.start_time = time.time()
```

---

## CORS CONFIGURATION

Para permitir que dashboard acceda desde localhost:3000:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## TESTING

### Verificar endpoints manualmente:

```bash
# Health
curl http://localhost:8791/api/health

# Learning
curl http://localhost:8791/api/learning/status

# Memory
curl http://localhost:8791/api/memory/stats

# Recent activity
curl http://localhost:8791/api/recent-activity?limit=5
```

### Response esperado:

Todos los endpoints deben retornar JSON válido con status code 200.

---

## ACTUALIZACIÓN EN TIEMPO REAL

### Opción 1: Polling (Actual)

Dashboard hace fetch cada 2-3 segundos.

**Pros:** Simple de implementar
**Cons:** Más carga en servidor

### Opción 2: WebSocket (Futuro)

Conexión persistente para updates en tiempo real.

```python
from fastapi import WebSocket

@app.websocket("/ws/dashboard")
async def websocket_dashboard(websocket: WebSocket):
    await websocket.accept()
    
    try:
        while True:
            # Enviar datos cada segundo
            data = {
                "health": await get_health(),
                "learning": await get_learning_status(),
                "timestamp": datetime.now().isoformat()
            }
            await websocket.send_json(data)
            await asyncio.sleep(1)
    except:
        pass
```

**Cliente React:**

```javascript
useEffect(() => {
  const ws = new WebSocket('ws://localhost:8791/ws/dashboard');
  
  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    setSystemData(data);
  };
  
  return () => ws.close();
}, []);
```

---

## PRIORIDAD DE IMPLEMENTACIÓN

1. ✅ **CRÍTICO:** `/api/health` - Básico para monitoreo
2. ✅ **ALTO:** `/api/learning/status` - Muestra progreso
3. ✅ **ALTO:** `/api/memory/stats` - Monitoreo de memoria
4. ⚠️ **MEDIO:** `/api/recent-activity` - Timeline de eventos
5. ⚠️ **MEDIO:** `/api/system/health` - Recursos del sistema
6. ℹ️ **BAJO:** `/api/vision/status` - Nice to have
7. ℹ️ **BAJO:** `/api/learning/tutor` - Info adicional

---

**Implementar endpoints en orden de prioridad para tener dashboard funcional rápidamente.**
