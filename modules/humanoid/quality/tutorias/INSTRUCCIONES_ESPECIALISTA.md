# 📋 SISTEMA DE TUTORÍAS Y VISITAS - ATLAS

## ⚠️ ATENCIÓN: LECTURA OBLIGATORIA PARA TODO ESPECIALISTA

**Si estás leyendo esto, significa que vas a trabajar con ATLAS.**

Antes de hacer CUALQUIER modificación, configuración o instrucción al sistema, **DEBES**:

1. **Registrarte** como especialista
2. **Iniciar una visita** formal
3. **Documentar** tu trabajo
4. **Firmar** tu informe

---

## 🔴 ¿POR QUÉ ES OBLIGATORIO?

- **Trazabilidad**: Necesitamos saber quién hizo qué cambios
- **Responsabilidad**: Tu firma digital queda registrada
- **Calidad**: Tus evaluaciones ayudan a mejorar el sistema
- **Continuidad**: Tus recomendaciones guían el desarrollo futuro

---

## 🚀 CÓMO EMPEZAR

### Opción 1: Dashboard (Recomendado)

1. Abre el navegador en: **http://127.0.0.1:8791**
2. Ve al tab **"📋 Tutorías"**
3. Sigue las instrucciones en pantalla

### Opción 2: API Directa

```bash
# 1. Registrarte como especialista
curl -X POST http://127.0.0.1:8791/tutorias/especialistas \
  -H "Content-Type: application/json" \
  -d '{
    "nombre": "Tu Nombre Completo",
    "rol": "Tu Rol (Arquitecto, Developer, QA, etc.)",
    "especialidad": "Tu Área (Vision, NLP, Robotics, etc.)",
    "email": "tu@email.com"
  }'

# Respuesta: Tu firma digital única
# {"ok":true,"especialista_id":"abc123","firma_digital":"04cfeb8d..."}

# 2. Iniciar visita
curl -X POST http://127.0.0.1:8791/tutorias/visitas \
  -H "Content-Type: application/json" \
  -d '{
    "tipo": "tutoria",
    "especialista_id": "TU_ID_AQUI",
    "motivo": "Descripción de lo que vas a hacer",
    "modulos_revisados": ["brain", "vision"]
  }'

# 3. Al terminar, finalizar con informe
curl -X POST http://127.0.0.1:8791/tutorias/visitas/VISITA_ID/finalizar \
  -H "Content-Type: application/json" \
  -d '{
    "titulo": "Informe de mi trabajo",
    "resumen": "Resumen ejecutivo",
    "contenido": "Detalle de lo realizado...",
    "evaluaciones": [
      {"aspecto": "Rendimiento", "nivel": "BUENO", "puntuacion": 4, "comentario": "Ok"}
    ],
    "recomendaciones": [
      {"titulo": "Mejorar X", "descripcion": "Detalle", "modulo_afectado": "vision", "prioridad": "MEDIA"}
    ]
  }'
```

---

## 📊 TIPOS DE VISITA

| Tipo | Emoji | Cuándo usar |
|------|-------|-------------|
| `tutoria` | 📚 | Enseñar o configurar ATLAS |
| `revision` | 🔍 | Inspeccionar código o funcionamiento |
| `auditoria` | 📊 | Evaluación formal de calidad |
| `capacitacion` | 🎓 | Entrenar al sistema |
| `mantenimiento` | 🔧 | Correcciones y ajustes |
| `emergencia` | 🚨 | Reparación urgente |
| `seguimiento` | 📌 | Verificar mejoras previas |

---

## ⭐ NIVELES DE EVALUACIÓN

| Nivel | Puntuación | Significado |
|-------|------------|-------------|
| EXCELENTE | 5 | Supera expectativas |
| BUENO | 4 | Cumple bien |
| ACEPTABLE | 3 | Funciona pero mejorable |
| MEJORABLE | 2 | Necesita atención |
| CRITICO | 1 | Requiere acción inmediata |

---

## 🔥 PRIORIDADES DE RECOMENDACIÓN

| Prioridad | Cuándo usar |
|-----------|-------------|
| CRITICA | Seguridad, errores graves, bloqueos |
| ALTA | Funcionalidad importante afectada |
| MEDIA | Mejora significativa de calidad |
| BAJA | Nice-to-have, optimizaciones |
| OPCIONAL | Ideas para el futuro |

---

## 📁 UBICACIÓN DE DATOS

- **Base de datos**: `data/quality/tutorias.db`
- **Reportes generados**: `data/reports/tutorias/`
- **POT relacionado**: `modules/humanoid/quality/pots/specialist_visit.py`

---

## 🔗 ENDPOINTS API

| Endpoint | Método | Descripción |
|----------|--------|-------------|
| `/tutorias/especialistas` | GET | Listar especialistas |
| `/tutorias/especialistas` | POST | Registrar especialista |
| `/tutorias/visitas` | GET | Listar visitas |
| `/tutorias/visitas` | POST | Iniciar visita |
| `/tutorias/visitas/{id}/finalizar` | POST | Finalizar con informe |
| `/tutorias/recomendaciones` | GET | Ver recomendaciones |
| `/tutorias/seguimientos` | GET | Ver seguimientos |
| `/tutorias/estadisticas` | GET | Estadísticas |
| `/tutorias/dashboard` | GET | Generar dashboard HTML |

---

## ✍️ FIRMA DIGITAL

Cada informe queda firmado con:
- **Firma del especialista**: Hash único de 16 caracteres
- **Hash de verificación**: SHA-256 del contenido del informe
- **Timestamp**: Fecha y hora exacta de la firma

Esta firma es **inmutable** y queda registrada permanentemente.

---

## 📞 ¿DUDAS?

Consulta el POT completo:
```bash
curl http://127.0.0.1:8791/pots/specialist_visit
```

O revisa la documentación en el dashboard.

---

**RECUERDA**: Sin registro, sin visita documentada = sin cambios permitidos.

*Sistema de Calidad ATLAS v3.8.0*
