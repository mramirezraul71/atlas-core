# 🎨 ATLAS NEXUS DASHBOARD - DOCUMENTACIÓN

## 📋 OVERVIEW

Dashboard profesional y minimalista para monitorear ATLAS NEXUS en tiempo real.

**Características:**
- ✅ Diseño moderno y minimalista
- ✅ Actualización en tiempo real
- ✅ Tema oscuro profesional
- ✅ Visualizaciones claras y funcionales
- ✅ Totalmente responsive
- ✅ Código limpio y modular

---

## 🎯 COMPONENTES PRINCIPALES

### 1. HEADER
- Logo ATLAS con gradiente cyan-blue
- Estado general del sistema
- Reloj en tiempo real

### 2. CORE SERVICES (3 tarjetas)
- **PUSH** (Puerto 8791) - API principal
- **NEXUS** (Puerto 8000) - Core system
- **ROBOT** (Puerto 8002) - Robot interface

**Métricas por servicio:**
- Status (Active/Inactive)
- Uptime
- Load % (barra de progreso animada)

### 3. ACTIVE LEARNING SESSION
Sección destacada con gradiente violeta que muestra:
- Lección actual del día
- Progreso % con barra animada
- Tasks completadas / Total
- Tutor score
- Uncertainty rate
- Learning speed

### 4. PERFORMANCE METRICS (4 cards)
- Success Rate (%)
- Average Confidence (%)
- Episodes Today
- Consolidations

**Cada card incluye:**
- Icono con gradiente
- Valor principal
- Label
- Trend (opcional)

### 5. RECENT ACTIVITY
Timeline de últimas 5 acciones del robot:
- Timestamp
- Descripción de acción
- Status (success/warning/error)
- Confidence score

### 6. MEMORY SYSTEMS (3 tarjetas)
**Episodic Memory:**
- Total episodios
- Nuevos hoy
- Backend: SQLite

**Semantic Memory:**
- Total embeddings
- Conceptos
- Backend: FAISS

**Knowledge Base:**
- Conceptos
- Skills
- Rules

### 7. AI TUTOR STATUS
- Estado (Active/Inactive)
- Modelo (Claude Opus 4.5)
- Lecciones completadas
- Nivel del robot
- Average score
- Next review countdown

### 8. VISION SYSTEM
Estado de componentes de visión:
- YOLO Detection
- Depth Estimation (MiDaS)
- Scene Understanding (LLaVA)
- Multi-Camera Fusion

### 9. SYSTEM HEALTH
Barras de progreso para:
- CPU Usage
- Memory Usage
- GPU Usage
- Storage

---

## 🚀 INSTALACIÓN Y USO

### OPCIÓN 1: Standalone HTML (Más Rápido)

**Archivo:** `atlas_dashboard_standalone.html`

```bash
# Simplemente abre el archivo en navegador
# Doble click o:
open atlas_dashboard_standalone.html  # macOS
start atlas_dashboard_standalone.html # Windows
```

**Ventajas:**
- ✅ No requiere instalación
- ✅ No requiere build
- ✅ Funciona inmediatamente
- ✅ Perfecto para demos

**Desventajas:**
- ⚠️ No se conecta a APIs reales (datos simulados)
- ⚠️ Menos eficiente que versión React compilada

---

### OPCIÓN 2: React Component (Producción)

**Archivo:** `atlas_dashboard.jsx`

#### 1. Crear proyecto React (si no existe)

```bash
npx create-react-app atlas-dashboard
cd atlas-dashboard
```

#### 2. Copiar componente

```bash
# Copiar atlas_dashboard.jsx a src/
cp atlas_dashboard.jsx src/AtlasDashboard.jsx
```

#### 3. Instalar Tailwind CSS

```bash
npm install -D tailwindcss
npx tailwindcss init
```

**Configurar `tailwind.config.js`:**

```javascript
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
  ],
  theme: {
    extend: {},
  },
  plugins: [],
}
```

**Agregar a `src/index.css`:**

```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

#### 4. Instalar Lucide Icons

```bash
npm install lucide-react
```

#### 5. Importar en App.js

```javascript
import AtlasDashboard from './AtlasDashboard';

function App() {
  return <AtlasDashboard />;
}

export default App;
```

#### 6. Ejecutar

```bash
npm start
```

Dashboard disponible en: `http://localhost:3000`

---

## 🔌 INTEGRACIÓN CON ATLAS APIs

### Conectar a APIs Reales

Modificar el componente para hacer fetch a endpoints:

```javascript
useEffect(() => {
  const fetchData = async () => {
    try {
      // Fetch services status
      const pushRes = await fetch('http://localhost:8791/api/health');
      const nexusRes = await fetch('http://localhost:8000/api/health');
      const robotRes = await fetch('http://localhost:8002/api/health');
      
      // Fetch learning status
      const learningRes = await fetch('http://localhost:8791/api/learning/status');
      
      // Fetch memory stats
      const memoryRes = await fetch('http://localhost:8791/api/memory/stats');
      
      // Update state
      setSystemData({
        services: {
          push: await pushRes.json(),
          nexus: await nexusRes.json(),
          robot: await robotRes.json()
        },
        learning: await learningRes.json(),
        memory: await memoryRes.json()
      });
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };
  
  // Initial fetch
  fetchData();
  
  // Update every 2 seconds
  const interval = setInterval(fetchData, 2000);
  
  return () => clearInterval(interval);
}, []);
```

### APIs Requeridas

Dashboard espera estos endpoints en ATLAS:

```
GET /api/health                    → Status de servicio
GET /api/learning/status           → Estado de lección actual
GET /api/memory/stats              → Estadísticas de memoria
GET /api/metrics/performance       → Métricas de performance
GET /api/recent-activity           → Últimas acciones
GET /api/tutor/status              → Estado del tutor IA
GET /api/vision/status             → Estado de sistema de visión
GET /api/system/health             → Salud del sistema
```

---

## 🎨 PERSONALIZACIÓN

### Cambiar Colores

Los colores principales están definidos con clases de Tailwind:

```javascript
// Cyan/Blue (Primario)
from-cyan-500 to-blue-600

// Violet/Fuchsia (Learning)
from-violet-500 to-fuchsia-500

// Emerald/Teal (Tutor)
from-emerald-500 to-teal-500

// Modificar según necesidad
```

### Agregar Nuevas Secciones

Template para nueva card:

```javascript
<div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-2xl p-6 shadow-xl">
  <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
    <Icon className="w-5 h-5 text-cyan-400" />
    <span>Título</span>
  </h2>
  
  {/* Contenido */}
</div>
```

### Modificar Layout

Grid de 12 columnas:
- Columna izquierda: `col-span-8` (8/12)
- Columna derecha: `col-span-4` (4/12)

Para layout diferente:

```javascript
// 50/50 split
<div className="col-span-6">...</div>
<div className="col-span-6">...</div>

// 3 columnas iguales
<div className="col-span-4">...</div>
<div className="col-span-4">...</div>
<div className="col-span-4">...</div>
```

---

## 📊 DATOS SIMULADOS VS REALES

### Actualmente Simulados

El dashboard standalone incluye datos de ejemplo:

```javascript
services: {
  push: { status: 'healthy', uptime: '47h 23m', port: 8791, load: 23 },
  nexus: { status: 'healthy', uptime: '47h 23m', port: 8000, load: 67 },
  robot: { status: 'healthy', uptime: '47h 23m', port: 8002, load: 45 }
}
```

**Load se actualiza cada 3 segundos** con variación aleatoria para demo.

### Integrar Datos Reales

Ver sección "Integración con ATLAS APIs" arriba.

---

## 🎯 FEATURES DESTACADAS

### 1. Animaciones Sutiles
- Pulso en indicadores activos
- Transiciones suaves en barras de progreso
- Hover effects en cards

### 2. Gradientes Profesionales
- Colores coherentes con tema ATLAS
- Gradientes sutiles para profundidad
- Backdrop blur para modernidad

### 3. Tipografía
- Inter font (clean y profesional)
- Font mono para valores numéricos
- Jerarquía clara de tamaños

### 4. Responsive Design
- Grid adaptable
- Mobile-friendly (con ajustes)
- Flexible a diferentes tamaños

### 5. Iconografía
- Lucide React icons (consistentes)
- Tamaños apropiados
- Colores temáticos

---

## 🔧 TROUBLESHOOTING

### Dashboard no se ve bien

**Problema:** Tailwind no está cargando
**Solución:** Verificar que `tailwind.config.js` esté configurado correctamente

### Iconos no aparecen

**Problema:** lucide-react no instalado
**Solución:** `npm install lucide-react`

### Datos no se actualizan

**Problema:** APIs no responden
**Solución:** Verificar que servicios ATLAS estén corriendo en puertos correctos

### Errores de CORS

**Problema:** Dashboard en puerto 3000, APIs en 8791/8000/8002
**Solución:** Configurar CORS en backend o usar proxy

```javascript
// En package.json
"proxy": "http://localhost:8791"
```

---

## 📁 ESTRUCTURA DE ARCHIVOS

```
atlas-dashboard/
├── atlas_dashboard.jsx              # Componente React principal
├── atlas_dashboard_standalone.html  # Versión standalone
├── README.md                         # Esta documentación
└── screenshots/                      # (opcional) Screenshots del dashboard
```

---

## 🚀 PRÓXIMOS PASOS

1. **Conectar a APIs reales** de ATLAS
2. **Agregar gráficos** (Charts.js / Recharts)
3. **Implementar WebSocket** para updates en tiempo real
4. **Agregar alertas** para eventos críticos
5. **Crear vista de logs** detallados
6. **Implementar modo light/dark** toggle
7. **Agregar export de métricas** (CSV/PDF)

---

## 💡 TIPS DE USO

- **Modo demo:** Usa versión standalone para presentaciones
- **Desarrollo:** Usa versión React con hot reload
- **Producción:** Build optimizado con `npm run build`
- **Monitoreo 24/7:** Deploy en servidor con auto-refresh

---

## 📞 SOPORTE

Para dudas sobre integración con ATLAS:
- Revisar documentación de APIs en `/docs/API_REFERENCE.md`
- Verificar endpoints disponibles con `curl http://localhost:8791/health`

---

**Diseñado específicamente para ATLAS NEXUS v1.0**
**Última actualización:** 2025-02-15
