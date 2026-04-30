# ATLAS Options Engine — Arquitectura de Observabilidad
## Documento de Diseño (Paper Mode · Pre-Live)

**Fecha:** 19 de abril de 2026  
**Base de referencia:** Auditoría v3 (`options_audit_v3_full.md`)  
**Alcance:** Paper mode. Sin cambios de código. Sin redefinir `8791/ui`. Preparado para escalar a live.  
**Autor:** Computer Agent — rol Arquitecto de Observabilidad

---

## Tabla de Contenidos

1. [Rol de la UI Central `8791/ui` y Visión de Observabilidad](#1-rol-de-la-ui-central-8791ui-y-visión-de-observabilidad)
2. [Arquitectura de Dashboards y Vistas Propuesta](#2-arquitectura-de-dashboards-y-vistas-propuesta)
3. [Diseño Detallado: Options Engine — Health](#3-diseño-detallado-options-engine--health)
4. [Diseño Detallado: Options Engine — Signals & Intent](#4-diseño-detallado-options-engine--signals--intent)
5. [Diseño Detallado: Options Engine — Paper Performance](#5-diseño-detallado-options-engine--paper-performance)
6. [Integración con ATLAS-QUANT PRO y la UI Central](#6-integración-con-atlas-quant-pro-y-la-ui-central)
7. [Integración con Code Quant, OptionStrat y Escaner Quant](#7-integración-con-code-quant-optionstrat-y-escaner-quant)
8. [Estabilidad, Resiliencia Visual y Sentinelas](#8-estabilidad-resiliencia-visual-y-sentinelas)
9. [Especificación Técnica para Implementación (Cursor)](#9-especificación-técnica-para-implementación-cursor)

---

## 1. Rol de la UI Central `8791/ui` y Visión de Observabilidad

### 1.1 Rol actual de `http://localhost:8791/ui`

`8791/ui` es el **hub de comando y control de ATLAS**. No es un dashboard de métricas — es el punto desde el que un operador navega, activa módulos, y obtiene el estado general del sistema antes de entrar a cualquier detalle.

Según la auditoría v3, `atlas_adapter/atlas_http_api.py` sirve en este puerto el dashboard del orquestador: comandos de alto nivel (start/stop ATLAS, modo paper/live, override de señales), proxy reverso hacia Code-Quant UI vía `/quant-ui`, y estado general del sistema Atlas completo.

**Lo que un operador encuentra hoy al abrir `8791/ui`:**

| Categoría | Qué hay | Calidad actual |
|-----------|---------|----------------|
| Trading general | Estado del motor, modo paper/live, equity | Operacional |
| Acceso a Code Quant | Enlace / proxy `/quant-ui` | Funciona, puerto a veces documentado mal (8792) |
| Escaner Quant | Acceso indirecto via Code Quant | Sin bloque propio en hub |
| OptionStrat | Acceso directo a `:8795/options/ui` | Sin estado resumido en hub |
| Grafana | Enlace manual | Sin estado embebido |
| Monitor / Governance | Tarjetas o módulos | Estado operacional variable |

**Lo que un operador debería encontrar al abrir el día (visión ideal):**

Un operador que llega a las 8:50 AM antes de la apertura necesita saber, en 10 segundos, tres cosas:
1. ¿El sistema está arriba y sano?
2. ¿El Options Engine está en GO o NO-GO para hoy?
3. ¿Dónde voy para el detalle?

Hoy ninguna de esas tres preguntas tiene una respuesta visible y consolidada en `8791/ui`.

**Cómo organiza el acceso hoy — evaluación:**

| Área | Organización actual | Problema |
|------|---------------------|----------|
| Trading | Aceptable | Datos de opciones no separados |
| Scanner | Indirecto (dentro de Code Quant) | Sin contexto de opciones flow |
| Opciones | Solo enlace a OptionStrat | Sin estado de sesión, sin briefing |
| Dashboards | Enlace plano a Grafana | Sin contextualización por rol |
| Herramientas auxiliares | ATLAS Directo, Rauli Vision, Approvals | Sin integración con states del engine |

### 1.2 Cómo debe encajar el Options Engine en `8791/ui`

El Options Engine no debe dominar `8791/ui` — ATLAS es más que opciones. Pero sí debe tener presencia visible, proporcional a su importancia en la fase actual (paper, objetivo 100 trades).

**Visión de conjunto — qué se ve dónde:**

```
8791/ui (hub central)
│
├── BLOQUE "OPTIONS ENGINE" (pequeño, high-level)
│   ├── Badge: GO / NO-GO / DEGRADADO (paper only)
│   ├── Stat: Trades paper hoy: 3 | Total: 23/100
│   └── Enlace → "Options Health" (Grafana)
│
├── Enlace → Code Quant (:8795/ui)
│   └── Vista incluye bloque "Options Session" (briefing diario)
│
├── Enlace → OptionStrat (:8795/options/ui)
│   └── Vista enriquecida con planner + journal
│
├── Enlace → Escaner Quant (view dentro de :8795/ui)
│   └── Columnas de options flow en tabla de candidatos
│
└── Enlace → Grafana (:3000)
    ├── Options Engine — Health
    ├── Options Engine — Signals & Intent
    └── Options Engine — Paper Performance
```

**Principio rector:** `8791/ui` muestra el **estado de semáforo** del Options Engine. Todo el detalle vive en Grafana o en los módulos especializados. El hub nunca reemplaza la profundidad — la enlaza.

---

## 2. Arquitectura de Dashboards y Vistas Propuesta

### 2.1 Las Cuatro Capas de Observabilidad

```
CAPA 1 — HUB CENTRAL (8791/ui)
  Propósito: Estado global, navegación, GO/NO-GO
  Usuario: Operador al inicio del día, cada mañana
  Frecuencia: 1× al día (inicio) + spot checks

CAPA 2 — DASHBOARDS GRAFANA (3 dashboards options)
  Propósito: Observabilidad profunda, diagnóstico, performance
  Usuario: Operador durante sesión + desarrollador
  Frecuencia: Intradía continuo (Health), diaria (Performance), semanal (análisis)

CAPA 3 — VISTAS INTERNAS ATLAS (Code Quant, OptionStrat, Escaner)
  Propósito: Operación activa, construcción de estrategias, candidatos
  Usuario: Operador en trading activo
  Frecuencia: Intradía, cada ciclo del scanner (3 min)

CAPA 4 — ATLAS-QUANT PRO GRAFANA (dashboard general)
  Propósito: Vista unificada del sistema completo con sección opciones
  Usuario: Operador/desarrollador en revisión de fin de día
  Frecuencia: Diaria (cierre), semanal (revisión de régimen)
```

### 2.2 Capa A — Hub Central `8791/ui`

**Objetivo:** Punto de entrada. Estado de semáforo. Navegación hacia profundidad.

**Usuario:** El operador que abre el sistema al inicio del día y quiere orientarse en <10 segundos.

**Frecuencia:** Diaria (apertura), spot checks intradía para confirmar que el sistema sigue arriba.

**Dependencia del OptionsPaperJournal:** Mínima — solo necesita el conteo de trades cerrados del día y el total histórico. No necesita leer el JSONL directamente; estos datos deben estar expuestos via API o métrica.

**Elementos que debe contener el bloque Options Engine en `8791/ui`:**

```
┌─────────────────────────────────────┐
│  OPTIONS ENGINE        [paper only] │
│                                     │
│  ● GO               IV Rank: 42     │
│  23 trades / 100 meta               │
│  Sesión: ACTIVA   WR: 58%           │
│                                     │
│  [Health ↗]  [Señales ↗]  [Perf ↗] │
└─────────────────────────────────────┘
```

### 2.3 Capa B — Dashboards Grafana (3 dashboards opciones)

| Dashboard | Rol | Frecuencia | Journal dependency |
|-----------|-----|------------|-------------------|
| **Options Engine — Health** | ¿El sistema está funcionando? ¿Puedo operar? | Continuo intradía | Alta — necesita conteos del journal |
| **Options Engine — Signals & Intent** | ¿Qué dice el mercado hoy? ¿Qué estrategia? | Diaria pre-apertura + cada ciclo | Media — necesita briefing y intent router |
| **Options Engine — Paper Performance** | ¿Cómo voy? ¿Se cumplen los criterios de la Fase 0? | Diaria (cierre) + semanal | Muy alta — es el dashboard del journal |

### 2.4 Capa C — Vistas Internas

| Vista | Rol | Frecuencia | Dependencia |
|-------|-----|------------|-------------|
| **Code Quant** | Bloque "Options Session" con briefing del día | Diaria apertura | SessionBriefingEngine via API |
| **OptionStrat** | Estrategias candidatas + journal comparison | Operación activa | Planner + Journal + AutoCloseEngine |
| **Escaner Quant** | Options flow en contexto de candidatos | Cada ciclo (3 min) | OptionsFlowProvider |

---

## 3. Diseño Detallado: Options Engine — Health

**UID sugerido:** `atlas-options-health`  
**Refresh:** 30 segundos  
**Tags:** `options`, `health`, `paper`  
**Objetivo:** Responder en una pantalla: "¿El Options Engine está vivo, operativo y haciendo lo que debe?"

### 3.1 Estructura de Filas

```
ROW 1 — STATUS GENERAL (siempre visible, tamaño grande)
ROW 2 — PIPELINE MODULES STATUS
ROW 3 — ACTIVIDAD DE HOY
ROW 4 — AUTO-CLOSE ENGINE
ROW 5 — ERRORES Y ALERTAS
```

### 3.2 Paneles Detallados

---

**PANEL H-01: GO / NO-GO Hoy**
- Tipo: `stat` con color de fondo
- Descripción: Muestra "GO" (verde), "NO-GO" (rojo) o "DEGRADADO" (amarillo) basado en si el último intent router completó con `allow_entry=true`, si el journal está activo, y si no hay errores críticos.
- Métrica conceptual: `atlas_options_session_go_nogo` (Gauge 1=GO, 0=NO-GO, 0.5=DEGRADADO)
- Pregunta que responde: "¿Puedo abrir posiciones paper hoy?"
- Prioridad: **P0**

---

**PANEL H-02: Estado del Pipeline (6 módulos)**
- Tipo: `table` con columna de estado por módulo
- Descripción: Una fila por módulo del pipeline. Columnas: Módulo | Último heartbeat | Estado | Latencia última ejecución.

| Módulo | Última ejecución | Estado | Latencia |
|--------|-----------------|--------|---------|
| SessionBriefingEngine | 09:05 | OK | 340ms |
| IVRankCalculator | 09:05 | OK (approx) | 1.2s |
| OptionsIntentRouter | 09:05 | OK | 12ms |
| PaperEntryPlanner | 09:05 | OK (size=None) | 8ms |
| PaperSessionOrchestrator | 09:05 | OK | 1.6s |
| AutoCloseEngine | 09:10 | OK | 22ms |

- Métrica conceptual: `atlas_options_pipeline_module_last_run_seconds{module}` (Gauge) + `atlas_options_pipeline_module_status{module}` (Gauge 1=OK, 0=ERROR, 0.5=WARN)
- Pregunta que responde: "¿Qué módulo está fallando o lento?"
- Prioridad: **P0**

---

**PANEL H-03: Journal Status**
- Tipo: `stat` (4 stats en fila)
- Stats: Eventos hoy | Sesiones completadas | Último evento hace | Tamaño del JSONL
- Métrica conceptual: `atlas_options_journal_events_today`, `atlas_options_journal_sessions_today`, `atlas_options_journal_last_event_age_seconds`, `atlas_options_journal_file_size_bytes`
- Pregunta que responde: "¿El journal está escribiendo? ¿Se están completando sesiones?"
- Prioridad: **P0**

---

**PANEL H-04: Trades Paper Hoy**
- Tipo: `stat` (3 stats horizontales)
- Stats: Abiertos ahora | Cerrados hoy | Cancelados/phantoms
- Métrica conceptual: `atlas_options_paper_trades_open`, `atlas_options_paper_trades_closed_today`, `atlas_options_paper_trades_phantom_today`
- Pregunta que responde: "¿Cuántas posiciones activas tengo ahora mismo?"
- Prioridad: **P0**

---

**PANEL H-05: Sesiones del Día (timeline)**
- Tipo: `time series` (estado del pipeline a lo largo del tiempo)
- Descripción: Línea continua mostrando el estado del orchestrador (GO=1, NO-GO=0) cada vez que corrió. Permite ver si hubo periodos sin sesión o caídas.
- Métrica conceptual: `atlas_options_session_go_nogo` como time series con resolución de 5 minutos
- Pregunta que responde: "¿El sistema estuvo activo durante toda la sesión de mercado?"
- Prioridad: **P1**

---

**PANEL H-06: Auto-Close Engine — Triggers Hoy**
- Tipo: `bar chart` (barras por motivo de trigger)
- Descripción: Barras mostrando cuántas decisiones de cierre tomó el AutoCloseEngine hoy, desglosadas por motivo: `credit_tp_50pct`, `credit_sl_2x`, `dte_gate_21d`, `breach_0dte`.
- Métrica conceptual: `atlas_options_autoclose_triggers_total{reason}` (Counter)
- Pregunta que responde: "¿Por qué se cerraron las posiciones paper hoy? ¿El TP está funcionando?"
- Prioridad: **P1**

---

**PANEL H-07: Auto-Close Engine — Posiciones Débito Sin Regla**
- Tipo: `stat` con alerta
- Descripción: Cuenta cuántas posiciones paper abiertas actualmente son de tipo débito (sin regla de PnL en el AutoCloseEngine). Si > 0, fondo amarillo con texto "X posiciones de débito sin stop automático".
- Métrica conceptual: `atlas_options_paper_debit_positions_no_stop` (Gauge)
- Pregunta que responde: "¿Tengo posiciones en riesgo sin gestión automática?"
- Prioridad: **P0** (gap crítico documentado en v3)

---

**PANEL H-08: Errores Operacionales — Últimas 6 horas**
- Tipo: `time series` (counter de errores por tipo)
- Descripción: Líneas separadas por tipo de error: IV fallback activado, pipeline exception, journal write error, AutoClose evaluation error.
- Métrica conceptual: `atlas_options_errors_total{type}` (Counter)
- Pregunta que responde: "¿Está el sistema degradándose silenciosamente?"
- Prioridad: **P0**

---

**PANEL H-09: IV Rank Quality Flag**
- Tipo: `stat` (3 states)
- Descripción: Muestra la calidad de la última lectura de IVRankCalculator: "OK", "APPROX" (amarillo), "INSUFFICIENT" (rojo) + nota "(RV proxy, no IV clásica)".
- Métrica conceptual: `atlas_options_iv_rank_quality` (Gauge 2=ok, 1=approx, 0=insufficient)
- Pregunta que responde: "¿Puedo confiar en el IV Rank que estoy usando hoy?"
- Prioridad: **P1** (crítico para interpretar briefings)

---

**PANEL H-10: OptionsPaperJournal — Integridad**
- Tipo: `stat` con threshold
- Descripción: Tiempo desde el último evento escrito al journal. Si > 15 minutos durante horario de mercado, fondo rojo "JOURNAL ESTANCADO".
- Métrica conceptual: `atlas_options_journal_last_write_age_seconds` (Gauge)
- Thresholds: 0–300s verde | 300–900s amarillo | >900s rojo
- Pregunta que responde: "¿El journal está vivo durante la sesión?"
- Prioridad: **P0**

---

### 3.3 Layout del Dashboard Health

```
┌──────────────────────────────────────────────────────────────┐
│  H-01 GO/NO-GO    H-03 Journal Stats (4 stats en fila)       │
│  [grande, verde]  [ hoy | sesiones | edad | tamaño ]         │
├──────────────────────────────────────────────────────────────┤
│  H-04 Trades Hoy  H-07 Débito sin stop   H-09 IV Quality     │
│  [abiertos|cerr.] [alerta si >0]         [OK/APPROX/INSUFF]  │
├──────────────────────────────────────────────────────────────┤
│  H-02 Pipeline Status (tabla completa, 6 filas)               │
├──────────────────────────────────────────────────────────────┤
│  H-05 Sesiones Timeline (24h)         H-06 AutoClose Triggers │
├──────────────────────────────────────────────────────────────┤
│  H-08 Errores Operacionales (time series, 6h)  H-10 Journal  │
│                                                Integridad     │
└──────────────────────────────────────────────────────────────┘
```

---

## 4. Diseño Detallado: Options Engine — Signals & Intent

**UID sugerido:** `atlas-options-signals`  
**Refresh:** 5 minutos  
**Tags:** `options`, `signals`, `intent`, `paper`  
**Objetivo:** Responder: "¿Qué está diciendo el mercado hoy sobre opciones? ¿Qué estrategia debería ejecutar?"

Este es el dashboard pre-apertura por excelencia. Se revisa a las 8:50 AM antes de que abra el mercado y se consulta durante el día cuando cambia el régimen.

### 4.1 Paneles Detallados

---

**PANEL S-01: IV Rank (Serie de Tiempo — 30 días)**
- Tipo: `time series` con banda de referencia
- Descripción: Línea del IV Rank calculado (RV proxy) durante los últimos 30 días. Banda horizontal en 20–30 (bajo, comprar volatilidad) y en 70–80 (alto, vender volatilidad). Punto actual marcado en rojo/verde.
- Métrica conceptual: `atlas_options_iv_rank_value` (Gauge, histórico)
- Nota: Incluir anotación "(RV envelope proxy)" para recordar que no es IV clásica.
- Pregunta que responde: "¿Estamos en un entorno de IV alta o baja comparado con el último mes?"
- Prioridad: **P0**

---

**PANEL S-02: IV Rank — Gauge Actual**
- Tipo: `gauge` (0–100)
- Descripción: El valor actual de IV Rank con zonas de color: 0–25 azul ("Compra vol"), 25–50 verde ("Neutral bajo"), 50–75 amarillo ("Neutral alto"), 75–100 rojo ("Vende vol").
- Métrica conceptual: `atlas_options_iv_rank_value` (último valor)
- Pregunta que responde: "Ahora mismo, ¿qué régimen de IV estoy en?"
- Prioridad: **P0**

---

**PANEL S-03: Expected Move Diario**
- Tipo: `stat` (3 stats en fila)
- Stats: EM 0DTE (±$X) | EM semanal (±$X) | EM mensual (±$X)
- Descripción: El expected move calculado por SessionBriefingEngine para cada horizonte. Fórmula: `spot × iv_annual × √(horizon/252)`.
- Métrica conceptual: `atlas_options_expected_move_usd{horizon}` (Gauge, labels: 0dte, weekly, monthly)
- Pregunta que responde: "¿Cuánto espera el mercado que se mueva el subyacente hoy/esta semana?"
- Prioridad: **P0**

---

**PANEL S-04: Gamma Regime — Actual y Tendencia**
- Tipo: `stat` + sparkline
- Descripción: El gamma regime actual (Positive / Negative / Neutral) con color (verde/rojo/gris) y mini-trend de las últimas 5 sesiones. Positive = mercado ancla, movimientos contenidos. Negative = mercado explosivo, movimientos amplificados.
- Métrica conceptual: `atlas_options_gamma_regime` (Gauge 1=positive, 0=neutral, -1=negative)
- Pregunta que responde: "¿El gamma favorece spreads definidos o estructuras de movimiento?"
- Prioridad: **P0**

---

**PANEL S-05: Gamma Bias % (OptionsFlowProvider)**
- Tipo: `time series` (intradiario, resolución 5 min)
- Descripción: El `gamma_bias_pct` calculado por `options_flow_provider.py`. Positivo = gamma de mercado net-long (dealers compran dips). Negativo = gamma net-short (dealers amplifican movimiento). Línea cero como referencia.
- Métrica conceptual: `atlas_options_gamma_bias_pct` (Gauge, actualmente en SQLite — requiere bridge, ver Sección 9)
- Pregunta que responde: "¿Los dealers de opciones están amplificando o amortiguando el movimiento intradía?"
- Prioridad: **P0**

---

**PANEL S-06: Put/Call Ratio (PCR)**
- Tipo: `time series` (intradiario) + stat con valor actual
- Descripción: PCR de `options_flow_provider.py`. Banda horizontal en 0.7 (neutral) y 1.2 (miedo extremo). Línea roja cuando PCR > 1.0 (más puts que calls — bearish sentiment).
- Métrica conceptual: `atlas_options_pcr` (Gauge)
- Pregunta que responde: "¿El flujo de opciones sugiere protección bearish o confianza alcista?"
- Prioridad: **P1**

---

**PANEL S-07: Estrategia Recomendada — Distribución del Día**
- Tipo: `pie chart` o `bar chart horizontal`
- Descripción: Distribución de las estrategias que el `OptionsIntentRouter` ha recomendado durante el día en cada sesión ejecutada. Categorías: iron_condor, credit_spread, long_call, long_put, butterfly, strangle, etc.
- Métrica conceptual: `atlas_options_recommended_strategy_count{strategy}` (Counter)
- Pregunta que responde: "¿Qué tipo de sesgo de estrategia ha tenido el sistema hoy?"
- Prioridad: **P1**

---

**PANEL S-08: DTE Mode — Distribución**
- Tipo: `bar chart` (3 barras: 0dte, weekly, monthly)
- Descripción: Cuántas sesiones del día han tenido cada DTE mode. Permite ver si el sistema está sesgado hacia 0DTE (alto riesgo) o mensual (bajo riesgo).
- Métrica conceptual: `atlas_options_dte_mode_count{mode}` (Counter, labels: 0dte, weekly, monthly)
- Pregunta que responde: "¿El sistema está operando con estructura de riesgo adecuada hoy?"
- Prioridad: **P1**

---

**PANEL S-09: Intent Router — GO vs NO-GO (histórico)**
- Tipo: `bar chart apilado` (diario, últimas 10 sesiones)
- Descripción: Por día, barras apiladas mostrando cuántas sesiones fueron GO, cuántas NO-GO (allow_entry=false), cuántas force_no_trade. Permite ver tendencia de bloqueos.
- Métrica conceptual: `atlas_options_intent_decision_count{decision}` (Counter, labels: go, no_go, force_no_trade)
- Pregunta que responde: "¿El sistema está bloqueándose más de lo esperado últimamente?"
- Prioridad: **P1**

---

**PANEL S-10: Señales Visuales — Count y Confianza**
- Tipo: `stat` (2 stats) + `time series`
- Stats: Breaches detectados hoy | Confianza promedio
- Time series: evolución del breach count durante el día
- Descripción: Salida del `VisualSignalAdapter`. Cuántas rupturas detectó y con qué nivel de confianza (cuando esté integrado al pipeline — ver nota).
- Métrica conceptual: `atlas_options_visual_breaches_total`, `atlas_options_visual_confidence_avg`
- Nota: Panel en estado DEGRADADO hasta que `VisualSignalAdapter` se integre al orchestrador (ver gap R-05 en v3).
- Pregunta que responde: "¿Hay señales visuales que cambian la decisión de entrada?"
- Prioridad: **P1** (P0 post-integración de VisualSignalAdapter)

---

**PANEL S-11: Régimen de Mercado ML + VIX**
- Tipo: `stat` dual
- Descripción: Panel izquierdo: régimen actual (Bear/Sideways/Bull) de `atlas_regime`. Panel derecho: estado VIX (normal/elevated/panic). Fondo rojo si VIX panic (scanner se corto-circuita en ese estado).
- Métrica conceptual: `atlas_regime`, `atlas_ocr_confidence_pct` (proxy VIX gate)
- Pregunta que responde: "¿El contexto macro justifica operar opciones hoy?"
- Prioridad: **P0**

---

**PANEL S-12: Heatmap de Estrategia Recomendada × Gamma Regime (histórico)**
- Tipo: `heatmap`
- Descripción: Eje X = gamma regime (positive/neutral/negative), Eje Y = estrategia recomendada. Color = frecuencia de aparición. Muestra qué combinaciones son más comunes y permite detectar si el router está siendo consistente.
- Métrica conceptual: `atlas_options_strategy_regime_matrix{strategy, gamma_regime}` (Counter con 2 labels)
- Pregunta que responde: "¿Es consistente el router? ¿Recomienda iron_condors cuando gamma es positivo?"
- Prioridad: **P2** (útil para calibración semanal)

---

### 4.2 Layout del Dashboard Signals & Intent

```
┌──────────────────────────────────────────────────────────────┐
│  S-02 IV Gauge   S-03 Expected Move    S-11 Régimen + VIX    │
│  [gauge 0-100]   [ 0dte | semanal | mensual ]  [stat doble]  │
├──────────────────────────────────────────────────────────────┤
│  S-01 IV Rank 30 días (time series, ancho completo)          │
├──────────────────────────────────────────────────────────────┤
│  S-04 Gamma Regime     S-05 Gamma Bias (time series)         │
│  [stat + sparkline]    [intradiario]                         │
├──────────────────────────────────────────────────────────────┤
│  S-06 PCR (time series + stat)   S-10 Señales Visuales       │
├──────────────────────────────────────────────────────────────┤
│  S-07 Estrategias Hoy   S-08 DTE Modes   S-09 GO/NO-GO hist  │
│  [pie]                  [bars]            [apilado 10d]       │
├──────────────────────────────────────────────────────────────┤
│  S-12 Heatmap Estrategia × Gamma Regime (ancho completo)     │
└──────────────────────────────────────────────────────────────┘
```

---

## 5. Diseño Detallado: Options Engine — Paper Performance

**UID sugerido:** `atlas-options-performance`  
**Refresh:** 5 minutos (durante sesión), manual (post-cierre)  
**Tags:** `options`, `performance`, `paper`, `fase0`  
**Objetivo:** Responder: "¿Cómo va el paper trading de opciones? ¿Estamos en camino a cumplir los criterios de la Fase 0?"

Meta de Fase 0: 100 trades limpios, WR > 40%, PF > 1.2.

### 5.1 Paneles Detallados

---

**PANEL P-01: Progreso Fase 0**
- Tipo: `gauge` o `bar gauge`
- Descripción: Barra de progreso visual: "23 / 100 trades limpios completados". Color basado en si el WR y PF actuales cumplen los umbrales.
- Métrica conceptual: `atlas_options_paper_closed_total` vs target 100; `atlas_options_paper_win_rate_pct`; `atlas_options_paper_profit_factor`
- Thresholds de color: Verde si WR>40% Y PF>1.2 | Amarillo si solo uno cumple | Rojo si ninguno cumple
- Pregunta que responde: "¿Estoy en camino para pasar a live según los criterios de Fase 0?"
- Prioridad: **P0**

---

**PANEL P-02: KPIs Globales (row de stats)**
- Tipo: `stat` (5 stats en fila)
- Stats: Win Rate % | Profit Factor | Total trades cerrados | PnL total paper ($) | PnL promedio por trade ($)
- Colores: WR verde si >40%, amarillo si 35–40%, rojo si <35%. PF verde si >1.2.
- Métrica conceptual: `atlas_options_paper_win_rate_pct`, `atlas_options_paper_profit_factor`, `atlas_options_paper_closed_total`, `atlas_options_paper_total_pnl_usd`, `atlas_options_paper_avg_pnl_usd`
- Pregunta que responde: "¿Cuál es la salud agregada del paper trading ahora mismo?"
- Prioridad: **P0**

---

**PANEL P-03: Equity Curve Paper**
- Tipo: `time series`
- Descripción: Curva de equity paper acumulada desde el inicio de la Fase 0 (post-purge). Línea principal de equity + banda de drawdown sombreada en rojo. Línea horizontal en capital inicial (referencia).
- Métrica conceptual: `atlas_options_paper_equity_usd` (Gauge acumulado), `atlas_options_paper_drawdown_pct`
- Pregunta que responde: "¿La cuenta paper está creciendo de forma consistente o hay drawdowns problemáticos?"
- Prioridad: **P0**

---

**PANEL P-04: Drawdown Rolling**
- Tipo: `time series` (solo drawdown, eje inverted)
- Descripción: Drawdown desde el máximo anterior, en porcentaje. Línea de alerta en -15%. Si el drawdown supera -20%, alerta visible en el panel.
- Métrica conceptual: `atlas_options_paper_drawdown_pct` (Gauge, negativo)
- Thresholds: 0–(-10%) verde | (-10%)–(-20%) amarillo | <(-20%) rojo
- Pregunta que responde: "¿Estoy en un drawdown peligroso que debería pausar la sesión?"
- Prioridad: **P0**

---

**PANEL P-05: PnL por Estrategia — Violin Plot o Box Plot**
- Tipo: `histogram` por categoría (aproximación en Grafana) o tabla con min/max/median/mean
- Descripción: Distribución de PnL por tipo de estrategia (iron_condor, credit_spread, long_call, etc.). Permite ver qué estrategias tienen mejor relación riesgo/recompensa y menor dispersión.
- Métrica conceptual: `atlas_options_paper_trade_pnl_usd{strategy}` (Histogram con label strategy)
- Pregunta que responde: "¿Qué estrategia es más rentable Y más consistente en papel?"
- Prioridad: **P1**

---

**PANEL P-06: PnL por Motivo de Cierre**
- Tipo: `bar chart` (horizontal)
- Descripción: PnL promedio agrupado por motivo de cierre del AutoCloseEngine: `credit_tp_50pct`, `credit_sl_2x`, `dte_gate_21d`, `breach_0dte`, `manual`. Permite ver si el TP da lo esperado y si el SL está calibrado.
- Métrica conceptual: `atlas_options_paper_pnl_by_close_reason{reason}` (Histogram)
- Pregunta que responde: "¿Las reglas de cierre automático generan los resultados esperados?"
- Prioridad: **P1**

---

**PANEL P-07: Win Rate por Estrategia (heatmap con gamma regime)**
- Tipo: `heatmap` o `table` con colores
- Descripción: Tabla cruzada: Estrategia × Gamma Regime. Valor = win rate %. Verde si >50%, amarillo 40–50%, rojo <40%. Muestra qué estrategias funcionan en qué regímenes.
- Métrica conceptual: `atlas_options_paper_win_rate_by_strategy_regime{strategy, gamma_regime}` (Gauge calculado)
- Pregunta que responde: "¿Los iron_condors realmente funcionan mejor en gamma positivo? ¿Hay estrategias que consistentemente fallan?"
- Prioridad: **P1**

---

**PANEL P-08: Trades Recientes (tabla)**
- Tipo: `table`
- Descripción: Los últimos 20 trades cerrados del journal paper. Columnas: Símbolo | Estrategia | Entry | Exit | PnL ($) | PnL (%) | Motivo cierre | DTE al cierre | Gamma regime.
- Métrica conceptual: Datos del OptionsPaperJournal via API o query directa
- Pregunta que responde: "¿Qué pasó exactamente en los trades recientes?"
- Prioridad: **P1**

---

**PANEL P-09: Trades Abiertos Ahora (tabla)**
- Tipo: `table` con semáforo de riesgo
- Descripción: Posiciones paper actualmente abiertas. Columnas: Símbolo | Estrategia | Entry | PnL actual ($) | DTE | Tipo (crédito/débito) | Stop activo | Próximo trigger.
- Columna "Stop activo" en rojo si el trade es de débito (sin regla PnL en AutoCloseEngine).
- Métrica conceptual: `atlas_options_paper_open_positions` via API
- Pregunta que responde: "¿Qué tengo abierto ahora mismo y cuál es el estado de riesgo de cada posición?"
- Prioridad: **P0**

---

**PANEL P-10: Detección de Phantoms / Ciclos Incompletos**
- Tipo: `stat` con alerta
- Descripción: Cuenta de sesiones que llegaron a `session_plan` pero no registraron ni `entry_execution` ni `close_execution` (sesiones "vacías" o abortadas a medias). Si > 0 en el día, fondo amarillo.
- Métrica conceptual: `atlas_options_paper_phantom_sessions_today` (Gauge)
- Pregunta que responde: "¿El pipeline está terminando sus ciclos o hay sesiones que se pierden a mitad?"
- Prioridad: **P1**

---

**PANEL P-11: Evolución Semanal de KPIs (tendencia)**
- Tipo: `time series` multi-línea
- Descripción: WR% y PF rolling semanal (calculados sobre ventana de 7 días deslizante). Líneas de referencia en WR=40% y PF=1.2 (metas Fase 0). Permite ver si el sistema está convergiendo hacia los criterios de paso a live.
- Métrica conceptual: `atlas_options_paper_win_rate_rolling_7d`, `atlas_options_paper_profit_factor_rolling_7d`
- Pregunta que responde: "¿Estoy mejorando semana a semana o hay estancamiento?"
- Prioridad: **P1**

---

**PANEL P-12: Día N de historial limpio post-purge**
- Tipo: `stat`
- Descripción: Número de días calendario desde el inicio de la Fase 0 (post-purge). Texto simple: "Día 12 de historial limpio". Sin colores — es informativo.
- Métrica conceptual: `atlas_options_paper_phase0_day_count` (Gauge, incrementa diariamente)
- Pregunta que responde: "¿Cuánto contexto limpio tenemos ya?"
- Prioridad: **P2**

---

### 5.2 Layout del Dashboard Paper Performance

```
┌──────────────────────────────────────────────────────────────┐
│  P-01 Progreso Fase 0 (gauge grande)   P-12 Día N post-purge │
├──────────────────────────────────────────────────────────────┤
│  P-02 KPIs Globales (5 stats en fila)                        │
│  [ WR% | PF | Trades | PnL total | PnL avg ]                 │
├──────────────────────────────────────────────────────────────┤
│  P-03 Equity Curve (time series, ancho completo)             │
├──────────────────────────────────────────────────────────────┤
│  P-04 Drawdown Rolling (eje invertido, con alerta -20%)      │
├──────────────────────────────────────────────────────────────┤
│  P-09 Trades Abiertos (tabla con semáforo riesgo)            │
├──────────────────────────────────────────────────────────────┤
│  P-05 PnL por Estrategia       P-06 PnL por Motivo Cierre    │
│  [histogram/box por estrategia]  [bar horizontal]            │
├──────────────────────────────────────────────────────────────┤
│  P-07 WR por Estrategia × Gamma Regime (heatmap)             │
├──────────────────────────────────────────────────────────────┤
│  P-08 Trades Recientes (tabla, 20 filas)                     │
├──────────────────────────────────────────────────────────────┤
│  P-11 KPIs Rolling 7d         P-10 Phantoms / Incompletos    │
└──────────────────────────────────────────────────────────────┘
```

---

## 6. Integración con ATLAS-QUANT PRO y la UI Central

### 6.1 `8791/ui` — Bloque de Estado Options Engine

**Sí debe mostrar un bloque.** Es el punto de entrada del día y el operador necesita saber el estado de opciones sin navegar a ningún otro lado primero.

**El bloque debe ser pequeño, denso y accionable.** No es un dashboard — es un panel de estado.

**Diseño del bloque:**

```
┌─────────────────────────────────────────────────────────────┐
│  OPTIONS ENGINE                                [paper only] │
│  ─────────────────────────────────────────────────────────  │
│  ● GO                   IV Rank: 42 (approx)                │
│                                                             │
│  Sesiones hoy: 3        Trades cerrados: 2 / 100 meta       │
│  Estrategia: Iron Condor (SPX weekly)                       │
│  WR: 58% · PF: 1.31     Último error: ninguno               │
│                                                             │
│  [Health ↗]   [Señales ↗]   [Performance ↗]   [Strat ↗]   │
└─────────────────────────────────────────────────────────────┘
```

**Especificación de cada campo:**

| Campo | Valor ejemplo | Color | Click destino |
|-------|--------------|-------|---------------|
| Badge GO/NO-GO | GO | Verde / Rojo / Amarillo | → Grafana Options Health |
| IV Rank | 42 (approx) | Blanco, "(approx)" en gris | → Grafana Signals & Intent |
| Sesiones hoy | 3 | Blanco | — |
| Trades cerrados | 2 / 100 meta | Blanco / azul si >50% meta | → Grafana Paper Performance |
| Estrategia actual | Iron Condor (SPX weekly) | Blanco | → OptionStrat |
| WR · PF | 58% · 1.31 | Verde si cumplen metas, rojo si no | → Grafana Paper Performance |
| Último error | "ninguno" o mensaje corto | Gris si ok, rojo si error | → Grafana Options Health |
| Botones | Health, Señales, Performance, Strat | Estilo hub ATLAS | Grafana / OptionStrat |

**Regla de representación visual:**
- Badge GO: fondo verde oscuro, texto blanco, punto verde parpadeante
- Badge NO-GO: fondo rojo oscuro, texto blanco, punto rojo fijo
- Badge DEGRADADO: fondo amarillo oscuro, texto negro, punto amarillo
- El bloque entero en gris con texto "Options Engine: sin datos" si el exporter está caído >5 minutos

### 6.2 `ATLAS-QUANT PRO` — Sección Options Engine

Añadir una nueva sección al final del dashboard `atlas_pro_2026.json` llamada **"OPTIONS ENGINE (paper)"** con 5 paneles stat en fila:

---

**Panel AQP-OE-01: Estado GO/NO-GO**
- Tipo: `stat`
- Métrica: `atlas_options_session_go_nogo`
- Valores: 1=GO (verde), 0=NO-GO (rojo), 0.5=DEGRADADO (amarillo)
- Texto: "GO" / "NO-GO" / "DEGRADADO"
- Tooltip: "Estado operativo del Options Engine. GO = briefing completo, intent allow_entry=true, journal activo."
- Umbrales: 1 → verde #3caf3c | 0.5 → amarillo #f0b429 | 0 → rojo #cf3a23

---

**Panel AQP-OE-02: Trades Paper Cerrados**
- Tipo: `stat`
- Métrica: `atlas_options_paper_closed_total`
- Texto: "23 / 100"
- Tooltip: "Trades paper completados desde el inicio de la Fase 0. Meta: 100 trades limpios con WR>40% y PF>1.2."
- Color: azul hasta 50%, verde >50%, amarillo si WR o PF no cumplen aún

---

**Panel AQP-OE-03: Win Rate Acumulado**
- Tipo: `stat`
- Métrica: `atlas_options_paper_win_rate_pct`
- Texto: "58.3%"
- Tooltip: "Win rate desde inicio de Fase 0. Meta mínima: 40%."
- Umbrales: ≥40 verde | 35–40 amarillo | <35 rojo

---

**Panel AQP-OE-04: Profit Factor**
- Tipo: `stat`
- Métrica: `atlas_options_paper_profit_factor`
- Texto: "1.31"
- Tooltip: "Profit Factor = ganancia bruta / pérdida bruta. Meta mínima: 1.2."
- Umbrales: ≥1.2 verde | 1.0–1.2 amarillo | <1.0 rojo

---

**Panel AQP-OE-05: Equity Paper**
- Tipo: `stat` con sparkline
- Métrica: `atlas_options_paper_equity_usd`
- Texto: "$10,340" (valor actual)
- Sparkline: últimas 30 sesiones
- Tooltip: "Capital paper acumulado del Options Engine desde inicio de Fase 0."
- Color: verde si equity > capital inicial, rojo si está por debajo

---

**Posición en ATLAS-QUANT PRO:** Añadir como Row 10 (después de RECONCILIATION, antes de ALERTS o al final). Nombre de fila: `OPTIONS ENGINE (paper) — Fase 0`.

---

## 7. Integración con Code Quant, OptionStrat y Escaner Quant

### 7.1 Code Quant — Bloque "Options Session"

**Ubicación:** Parte superior del dashboard de Code Quant (`:8795/ui`), debajo de los stats de equity/PnL, antes del panel de posiciones equity.

**Diseño del bloque:**

```
┌─────────────────────────────────────────────────────────────────┐
│  OPTIONS SESSION HOY                           [paper · GO]     │
│  ─────────────────────────────────────────────────────────────  │
│  Régimen: Gamma Negativo · IV: 42 (approx)                      │
│  Estrategia sugerida: Iron Condor SPX · DTE: weekly             │
│  Expected move 0DTE: ±$28.4 · Expected move semanal: ±$51.2     │
│                                                                 │
│  Sesiones completadas: 3 · Trades abiertos: 1 · Cerrados: 2    │
│  WR hoy: 100% (2/2)  ·  Posiciones débito sin stop: 0           │
│                                                                 │
│  [Ver en Grafana: Options Health ↗]  [Ir a OptionStrat ↗]      │
└─────────────────────────────────────────────────────────────────┘
```

**Contrato de datos concreto:**

| Campo | Fuente | Formato |
|-------|--------|---------|
| Badge GO/NO-GO | `atlas_options_session_go_nogo` | Texto + color |
| Régimen gamma | `atlas_options_gamma_regime` | Texto: "Gamma Positivo/Negativo/Neutral" |
| IV Rank | `atlas_options_iv_rank_value` + quality | "42 (approx)" — siempre con quality flag |
| Estrategia sugerida | Último briefing: `recommended_strategy` + `dte_mode` | Texto corto |
| Expected move | `atlas_options_expected_move_usd{horizon=0dte,weekly}` | "±$X.X" |
| Sesiones / Trades | `atlas_options_journal_sessions_today`, `atlas_options_paper_trades_open`, `atlas_options_paper_trades_closed_today` | Números |
| WR hoy | Calculado sobre trades del día | "N/N" |
| Posiciones débito sin stop | `atlas_options_paper_debit_positions_no_stop` | Número, rojo si >0 |

**Regla de colapso:** Si el Options Engine está en NO-GO o no hay datos, el bloque se muestra colapsado con solo: `OPTIONS SESSION · NO-GO — [Ver detalle ↗]`.

**Formato de los datos:** Todo texto, sin mini-gráficos. El bloque no debe exceder 4 líneas de información + botones. La profundidad va en Grafana.

### 7.2 OptionStrat — Vista Alineada con el Engine

La UI de OptionStrat (`:8795/options/ui`) debe evolucionar para reflejar el pipeline del nuevo engine. Estructura ideal en 5 elementos:

---

**Elemento OS-01: Banner de sesión activa**

En la parte superior de OptionStrat, antes de las 4 vistas, un banner horizontal con:
- Estrategia recomendada por el planner para esta sesión
- IV Rank actual con quality flag
- Gamma regime
- Botón: "Ver plan completo" → abre `POST /options/paper-session-plan` como modal

Esto hace que el operador siempre sepa en qué contexto está construyendo estrategias.

---

**Elemento OS-02: Vista Constructor — enriquecida**

En la vista Builder, añadir en el panel de resultados una columna adicional:
- "Sesión actual recomienda: Iron Condor" (texto informativo, no accionable)
- Si la estrategia construida coincide con la recomendada: badge verde "Alineada"
- Si no coincide: badge gris "Diverge del plan" (no es un error, solo información)

---

**Elemento OS-03: Vista Portfolio — columna PnL journal vs simulado**

En la tabla de estrategias abiertas, añadir una columna:
- "PnL simulado (portfolio_analyzer)" | "PnL journal (paper)" | "Delta"
- Si Delta > 5% en cualquier dirección: badge amarillo "Verificar"
- Permite detectar drift entre la simulación y el journal real

---

**Elemento OS-04: Vista Portfolio — motivo de cierre planeado**

Para cada posición abierta, añadir columna "Próximo trigger AutoClose":
- Ejemplo: "TP en $X.XX (50% crédito)" o "DTE gate: 8 días"
- Para posiciones de débito: "⚠ Sin regla automática — débito" en rojo

---

**Elemento OS-05: Historial — motivo real vs planeado**

En el historial de estrategias cerradas, añadir columnas:
- "Motivo cierre real" (del journal: `credit_tp_50pct`, `manual`, etc.)
- "¿Era el trigger esperado?" (Sí/No — compara con el plan de la entrada)

Estos 5 elementos hacen de OptionStrat una vista operacional integrada, no solo un constructor de estrategias. No requieren rediseño completo — son columnas, banners y campos adicionales sobre la estructura actual.

### 7.3 Escaner Quant — Options Flow en Contexto

**Integración de OptionsFlowProvider en la tabla de candidatos:**

La vista del Escaner Quant actualmente muestra candidatos con columnas de score, estrategia, timeframe, etc. Los datos de `options_flow_provider.py` deben incorporarse de la siguiente manera:

---

**Opción A (recomendada): Panel de contexto superior**

Antes de la tabla de candidatos, un panel horizontal con 4 stats globales del mercado (no por símbolo):

```
┌────────────────────────────────────────────────────────────┐
│  OPTIONS FLOW CONTEXT                                       │
│  Gamma Bias: +12.3%  IV Current: 18.4  PCR: 0.82           │
│  Régimen: Gamma Positivo · Modo scanner: hybrid            │
│  [Ver señales completas en Grafana ↗]                      │
└────────────────────────────────────────────────────────────┘
```

Estos 4 valores son de contexto de mercado — aplican a todos los candidatos, no solo a uno. Ponerlos como panel superior en lugar de repetirlos en cada fila es más limpio.

**Opción B: Columnas adicionales en la tabla (solo si hay opciones para ese símbolo)**

Para los candidatos que tienen datos de options flow disponibles (aquellos donde `_options_flow_provider.build_snapshot()` tuvo éxito), añadir al final de la tabla 3 columnas opcionales:

| Columna | Descripción | Formato |
|---------|-------------|---------|
| IV / HV | Ratio IV vs HV del símbolo | "1.32" (amarillo si >1.5) |
| Flow bias | Gamma bias del símbolo | "+8.1%" (verde si positivo en candidato alcista) |
| IV Rank | IV Rank del símbolo | "42 (approx)" |

Las columnas se muestran como "--" si el proveedor no tiene datos para ese símbolo en el ciclo actual (no como vacíos ni como error).

---

**Enlace desde Escaner a Signals & Intent:**

En el encabezado del panel de contexto superior, el botón "Ver señales completas en Grafana ↗" lleva directamente al dashboard `atlas-options-signals` en Grafana. Esto cierra el flujo: el operador ve candidatos en el scanner → entiende el contexto de opciones → profundiza en Grafana si quiere el análisis completo.

---

## 8. Estabilidad, Resiliencia Visual y Sentinelas

### 8.1 Reglas de UX para Datos Faltantes o Errores

La filosofía de ATLAS es minimalista y profesional. Los estados de error no deben ser silenciosos (como ocurre hoy con varios módulos) ni deben dominar la pantalla con mensajes alarmistas. La regla de oro: **siempre mostrar algo, nunca ocultar el problema.**

**Reglas por tipo de situación:**

---

**Situación 1: La métrica existe pero no hay datos todavía (inicio del día)**

- Mostrar el panel con valor "—" o "0" en gris claro
- Texto secundario en gris: "Sin datos aún · Sesión no iniciada"
- No usar rojo, no usar alerta
- Ejemplo: Panel "Trades paper cerrados" a las 8:00 AM muestra "0" + "Sin trades hoy"

---

**Situación 2: La métrica no existe en Prometheus (exporter no ha sido bridgeado aún)**

- Panel muestra fondo gris oscuro con texto: "Métrica no disponible · [nombre de la métrica]"
- Añadir nota tooltip: "Esta métrica requiere instrumentación. Ver Spec Técnica Sección 9."
- No mostrar "No data" vacío — siempre texto explicativo
- Color del borde del panel: amarillo (pendiente de instrumentación)

---

**Situación 3: El exporter está caído (Prometheus no recibe datos > N minutos)**

- Panel con fondo gris oscuro, borde rojo tenue
- Texto: "Datos detenidos · Último dato hace X min"
- No mostrar el último valor conocido como si fuera actual — puede inducir a error
- En Grafana: usar la opción "Show last known value" como opción explícita, no por defecto

---

**Situación 4: Prometheus o Grafana no responden**

- Grafana mostrará sus propios estados de error (panel gris "No data")
- En `8791/ui` y Code Quant: si los datos no llegan via API en 5s, mostrar:
  - Badge gris: "Sin conexión con métricas"
  - Timestamp del último refresh exitoso
  - No ocultar el bloque completo — mantenerlo visible pero marcado

---

**Situación 5: El journal lleva más de 15 minutos sin escribir durante horario de mercado**

- Panel H-10 (Journal Integridad) se vuelve rojo
- En `8791/ui`, el badge del Options Engine cambia a DEGRADADO (amarillo)
- No se cambia a NO-GO automáticamente — el journal estancado puede ser por falta de señales, no por error

---

**Mensajes de error recomendados (minimalismo ATLAS):**

| Situación | Texto sugerido | Icono |
|-----------|---------------|-------|
| Sin datos aún | "Sin datos · Sesión no iniciada" | · (punto gris) |
| Métrica no instrumentada | "Pendiente de instrumentación" | ○ (círculo vacío amarillo) |
| Datos detenidos | "Datos detenidos · Último: Xm" | ◐ (medio círculo rojo) |
| Sin conexión | "Sin conexión con métricas" | ✕ (x gris) |
| Error del módulo | "Módulo en error · Ver Health" | ● (punto rojo) |

### 8.2 Sentinelas de Observabilidad (5 paneles de alerta temprana)

Los sentinelas son paneles diseñados específicamente para detectar problemas antes de que se conviertan en fallos visibles. Se colocan en el dashboard **Options Engine — Health** como Row 0 (la primera fila visible).

---

**SENTINELA 1: Options Metrics Freshness**

- Nombre en panel: `"Options metrics — Frescura"`
- Qué mide: Tiempo transcurrido desde el último scrape de `atlas_options_*` en Prometheus
- Cómo: `time() - max(timestamp(atlas_options_iv_rank_value))` conceptualmente
- Thresholds: <5m verde | 5–15m amarillo | >15m rojo "MÉTRICAS ESTANCADAS"
- Por qué es crítico: Si el exporter de opciones cae, TODOS los paneles del dashboard muestran datos viejos sin indicación clara. Este panel hace el problema inmediatamente visible.

---

**SENTINELA 2: Journal Heartbeat**

- Nombre en panel: `"Journal — Latido"`
- Qué mide: `atlas_options_journal_last_write_age_seconds`
- Cómo: Gauge que incrementa segundo a segundo hasta el próximo write
- Thresholds: <300s (5min) verde | 300–900s amarillo | >900s rojo "JOURNAL ESTANCADO"
- Por qué es crítico: El journal puede fallar silenciosamente (permisos, disco lleno, path incorrecto). Sin este sentinela, no sabrías que los trades de la sesión no se están registrando.

---

**SENTINELA 3: AutoClose — Silencio Inesperado**

- Nombre en panel: `"AutoClose — Actividad esperada"`
- Qué mide: Si hay trades paper abiertos (`atlas_options_paper_trades_open > 0`) pero el AutoCloseEngine no ha emitido ninguna decisión en la última hora
- Cómo: Combinación de `atlas_options_paper_trades_open` y `time() - max(timestamp(atlas_options_autoclose_last_eval_ts))`
- Thresholds: Verde si trades=0 o si AutoClose evaluó en última 1h | Rojo si trades>0 Y AutoClose silencioso >1h
- Por qué es crítico: El AutoCloseEngine puede fallar o quedar sin invocar. Si tienes posiciones abiertas y AutoClose lleva 2 horas sin evaluar, estás sin protección automática.

---

**SENTINELA 4: Scanner sin Options Flow**

- Nombre en panel: `"Scanner — Options Flow activo"`
- Qué mide: `atlas_options_flow_provider_ready` (si el provider del scanner está disponible y respondiendo)
- Cómo: Gauge binario 1=listo, 0=degradado
- Thresholds: 1 verde "Options flow activo" | 0 amarillo "Scanner en modo proxy (sin options flow)"
- Por qué es crítico: Si OptionsFlowProvider falla, el scanner corre en modo `proxy_intradia` sin datos de opciones. Esto afecta el score de candidatos. El operador debe saber si está en modo degradado.

---

**SENTINELA 5: IV Rank Quality Degradation**

- Nombre en panel: `"IV Rank — Calidad"`
- Qué mide: `atlas_options_iv_rank_quality` (2=ok, 1=approx, 0=insufficient)
- Thresholds: 2 verde "OK" | 1 amarillo "APPROX (RV proxy)" | 0 rojo "INSUFICIENTE — briefing no confiable"
- Nota fija en el panel: "IV Rank usa RV envelope, no IV histórica clásica"
- Por qué es crítico: Un IV Rank en "insufficient_history" significa que el SessionBriefingEngine está usando el fallback de IV=50.0. En ese estado, la estrategia recomendada puede ser completamente incorrecta. El operador necesita saberlo antes de operar.

---

**Posición de los 5 sentinelas:**

```
┌─────────────────────────────────────────────────────────────┐
│  SENTINELAS DE SISTEMA (siempre visible, Row 0 del Health)   │
│                                                             │
│  [S1 Métricas frescas]  [S2 Journal latido]  [S3 AutoClose] │
│  [S4 Scanner options flow]  [S5 IV Rank quality]            │
└─────────────────────────────────────────────────────────────┘
```

Estos 5 paneles deben ser los primeros que ve el operador al abrir el dashboard Options Health. Si alguno está en rojo, hay que resolver ese problema antes de operar.

---

## 9. Especificación Técnica para Implementación (Cursor)

Esta sección es el contrato de implementación. Todo lo de aquí arriba es diseño; lo de aquí es la lista precisa de qué necesita existir en el código para que ese diseño funcione.

### 9.1 Lista Completa de Métricas a Implementar

---

#### Grupo A — Estado del Pipeline

| Nombre conceptual | Tipo | Labels | Componente emisor | Descripción |
|-------------------|------|--------|-------------------|-------------|
| `atlas_options_session_go_nogo` | Gauge | — | `PaperSessionOrchestrator` | 1=GO, 0=NO-GO, 0.5=DEGRADADO |
| `atlas_options_pipeline_module_last_run_seconds` | Gauge | `module` (briefing, iv_rank, intent, planner, orchestrator, autoclose) | Cada módulo | Timestamp Unix del último run exitoso |
| `atlas_options_pipeline_module_status` | Gauge | `module` | Cada módulo | 1=OK, 0=ERROR, 0.5=WARN |
| `atlas_options_pipeline_module_latency_ms` | Histogram | `module` | Cada módulo | Latencia en ms del último run |
| `atlas_options_errors_total` | Counter | `type` (iv_fallback, pipeline_exception, journal_write_error, autoclose_error) | Cada módulo | Incrementar en cada error por tipo |

---

#### Grupo B — IV Rank y Briefing

| Nombre conceptual | Tipo | Labels | Componente emisor | Descripción |
|-------------------|------|--------|-------------------|-------------|
| `atlas_options_iv_rank_value` | Gauge | `symbol` | `IVRankCalculator` | Valor de IV Rank 0–100 |
| `atlas_options_iv_rank_quality` | Gauge | `symbol` | `IVRankCalculator` | 2=ok, 1=approx, 0=insufficient |
| `atlas_options_expected_move_usd` | Gauge | `symbol`, `horizon` (0dte, weekly, monthly) | `SessionBriefingEngine` | Expected move en USD |
| `atlas_options_gamma_regime` | Gauge | `symbol` | `SessionBriefingEngine` | 1=positive, 0=neutral, -1=negative |
| `atlas_options_iv_current` | Gauge | `symbol` | `OptionsFlowProvider` | IV implícita actual (actualmente en SQLite) |

---

#### Grupo C — Intent Router

| Nombre conceptual | Tipo | Labels | Componente emisor | Descripción |
|-------------------|------|--------|-------------------|-------------|
| `atlas_options_intent_decision_count` | Counter | `decision` (go, no_go, force_no_trade) | `OptionsIntentRouter` | Incrementar en cada llamada a `route()` |
| `atlas_options_recommended_strategy_count` | Counter | `strategy` | `OptionsIntentRouter` | Por cada recomendación emitida |
| `atlas_options_dte_mode_count` | Counter | `mode` (0dte, weekly, monthly) | `SessionBriefingEngine` | Por cada briefing completado |
| `atlas_options_strategy_regime_matrix` | Counter | `strategy`, `gamma_regime` | `OptionsIntentRouter` | Combinación estrategia × régimen |
| `atlas_options_visual_breaches_total` | Counter | `severity` | `VisualSignalAdapter` | Breaches detectados |
| `atlas_options_visual_confidence_avg` | Gauge | — | `VisualSignalAdapter` | Confianza promedio de breaches |

---

#### Grupo D — Options Flow Provider

| Nombre conceptual | Tipo | Labels | Componente emisor | Descripción |
|-------------------|------|--------|-------------------|-------------|
| `atlas_options_gamma_bias_pct` | Gauge | `symbol` | `OptionsFlowProvider` | Gamma bias % (actualmente en SQLite) |
| `atlas_options_pcr` | Gauge | `symbol` | `OptionsFlowProvider` | Put/Call Ratio |
| `atlas_options_flow_provider_ready` | Gauge | — | `OpportunityScannerService` | 1=listo, 0=degradado/proxy mode |

---

#### Grupo E — Journal y Trades Paper

| Nombre conceptual | Tipo | Labels | Componente emisor | Descripción |
|-------------------|------|--------|-------------------|-------------|
| `atlas_options_journal_events_today` | Gauge | `event_type` (session_plan, entry_execution, close_decision, close_execution) | `OptionsPaperJournal` | Eventos escritos hoy por tipo |
| `atlas_options_journal_sessions_today` | Gauge | — | `OptionsPaperJournal` | Sesiones completadas hoy |
| `atlas_options_journal_last_write_age_seconds` | Gauge | — | `OptionsPaperJournal` | Segundos desde el último write |
| `atlas_options_journal_file_size_bytes` | Gauge | — | `OptionsPaperJournal` | Tamaño del JSONL |
| `atlas_options_paper_trades_open` | Gauge | — | `OptionsPaperJournal` | Posiciones paper abiertas ahora |
| `atlas_options_paper_trades_closed_today` | Gauge | — | `OptionsPaperJournal` | Trades cerrados hoy |
| `atlas_options_paper_trades_phantom_today` | Gauge | — | `OptionsPaperJournal` | Sesiones plan sin entry ni close |
| `atlas_options_paper_debit_positions_no_stop` | Gauge | — | `AutoCloseEngine` + Journal | Posiciones débito sin regla PnL |
| `atlas_options_paper_closed_total` | Counter | — | `OptionsPaperJournal` | Total histórico desde Fase 0 |

---

#### Grupo F — Paper Performance

| Nombre conceptual | Tipo | Labels | Componente emisor | Descripción |
|-------------------|------|--------|-------------------|-------------|
| `atlas_options_paper_equity_usd` | Gauge | — | `OptionsPaperJournal` / cálculo | Capital paper acumulado |
| `atlas_options_paper_drawdown_pct` | Gauge | — | Calculado | Drawdown desde máximo previo |
| `atlas_options_paper_win_rate_pct` | Gauge | — | Calculado | WR global desde Fase 0 |
| `atlas_options_paper_profit_factor` | Gauge | — | Calculado | PF global desde Fase 0 |
| `atlas_options_paper_avg_pnl_usd` | Gauge | — | Calculado | PnL promedio por trade |
| `atlas_options_paper_total_pnl_usd` | Gauge | — | Calculado | PnL total acumulado |
| `atlas_options_paper_trade_pnl_usd` | Histogram | `strategy`, `close_reason`, `gamma_regime`, `dte_mode` | `OptionsPaperJournal` | PnL por trade con 4 labels |
| `atlas_options_paper_win_rate_rolling_7d` | Gauge | — | Calculado | WR rolling 7 días |
| `atlas_options_paper_profit_factor_rolling_7d` | Gauge | — | Calculado | PF rolling 7 días |
| `atlas_options_paper_phase0_day_count` | Gauge | — | Script diario | Días desde inicio Fase 0 |
| `atlas_options_autoclose_triggers_total` | Counter | `reason` (credit_tp_50pct, credit_sl_2x, dte_gate_21d, breach_0dte) | `AutoCloseEngine` | Por cada decisión de cierre |
| `atlas_options_autoclose_last_eval_ts` | Gauge | — | `AutoCloseEngine` | Timestamp último `evaluate()` |
| `atlas_options_paper_win_rate_by_strategy_regime` | Gauge | `strategy`, `gamma_regime` | Calculado periódicamente | WR cruzado para heatmap |

---

### 9.2 Lista de Dashboards y Paneles

#### Dashboard 1: Options Engine — Health
- **UID:** `atlas-options-health`
- **Refresh:** 30s
- **Dependencias clave:** `OptionsPaperJournal`, exporter pipeline, `AutoCloseEngine` timestamps
- **Paneles:** H-01 a H-10 + 5 sentinelas (Row 0)
- **Query principal para sentinela Journal:** `time() - atlas_options_journal_last_write_age_seconds`
- **Query principal para H-01:** `atlas_options_session_go_nogo`
- **Query principal para H-06:** `increase(atlas_options_autoclose_triggers_total[24h])` by label `reason`

#### Dashboard 2: Options Engine — Signals & Intent
- **UID:** `atlas-options-signals`
- **Refresh:** 5min
- **Dependencias clave:** `IVRankCalculator`, `SessionBriefingEngine`, `OptionsFlowProvider` (bridge SQLite→Prometheus), `OptionsIntentRouter`
- **Paneles:** S-01 a S-12
- **Query principal para S-01:** `atlas_options_iv_rank_value` + range 30d
- **Query principal para S-07:** `increase(atlas_options_recommended_strategy_count[24h])` by label `strategy`
- **Query principal para S-12:** `atlas_options_strategy_regime_matrix` como heatmap

#### Dashboard 3: Options Engine — Paper Performance
- **UID:** `atlas-options-performance`
- **Refresh:** 5min
- **Dependencias clave:** `OptionsPaperJournal` reader (actualmente write-only — **requiere reader**), métricas de performance calculadas
- **Paneles:** P-01 a P-12
- **Query principal para P-03:** `atlas_options_paper_equity_usd` + range since Fase 0 start
- **Query principal para P-05:** `atlas_options_paper_trade_pnl_usd` histogram by label `strategy`
- **Query principal para P-07:** `atlas_options_paper_win_rate_by_strategy_regime` como tabla/heatmap

---

### 9.3 Cambios Mínimos en la UI Interna

#### `8791/ui` — Hub Central

| Bloque nuevo | Datos a mostrar | Fuente de datos | Enlace de click |
|-------------|-----------------|-----------------|-----------------|
| "OPTIONS ENGINE" card | GO/NO-GO badge, IV Rank, sesiones hoy, trades/meta, WR·PF, último error | `/options/paper-session-plan` result + Prometheus via API interna | GO badge → Grafana Health; trades → Grafana Performance; IV → Grafana Signals |
| 3 botones en la card | [Health ↗] [Señales ↗] [Performance ↗] | — | `http://localhost:3000/d/atlas-options-health`, etc. |

---

#### Code Quant (`:8795/ui`)

| Bloque nuevo | Datos a mostrar | Fuente de datos | Enlace |
|-------------|-----------------|-----------------|--------|
| "OPTIONS SESSION HOY" (bloque superior, colapsable) | GO/NO-GO, régimen, IV Rank+quality, estrategia sugerida, EM 0DTE+semanal, sesiones, trades, WR hoy, débito sin stop | `GET /options/briefing` + `GET /options/iv-rank` + journal API | [Ver en Grafana Health ↗] [Ir a OptionStrat ↗] |

---

#### OptionStrat (`:8795/options/ui`)

| Elemento nuevo | Ubicación | Datos | Fuente |
|----------------|-----------|-------|--------|
| Banner de sesión activa | Header superior (todas las vistas) | Estrategia recomendada, IV Rank, gamma regime | `GET /options/paper-session-plan` |
| Badge "Alineada/Diverge" en constructor | Panel de resultados de cálculo | Compara estrategia construida vs recomendada | Local, comparación client-side |
| Columna "PnL journal vs simulado" en Portfolio | Tabla de estrategias abiertas | PnL del journal JSONL vs `portfolio_analyzer.py` | API journal reader (nuevo) |
| Columna "Próximo trigger AutoClose" en Portfolio | Tabla de estrategias abiertas | `GET /options/close-candidates` | `/options/close-candidates` |
| Columna "Motivo cierre real" en Historial | Tabla de historial | `log_close_execution` event del journal | API journal reader (nuevo) |

---

#### Escaner Quant

| Elemento nuevo | Ubicación | Datos | Fuente |
|----------------|-----------|-------|--------|
| Panel de contexto superior "OPTIONS FLOW CONTEXT" | Encima de la tabla de candidatos | Gamma Bias global, IV Current, PCR, modo scanner | `GET /scanner/status` + OptionsFlowProvider state |
| Columnas adicionales opcionales (IV/HV, Flow Bias, IV Rank) | Tabla de candidatos | Por símbolo, cuando disponible | `_options_flow_provider.build_snapshot()` |
| Botón "Ver señales completas ↗" | Panel de contexto superior | — | `http://localhost:3000/d/atlas-options-signals` |

---

### 9.4 Dependencias Críticas que Deben Resolverse Primero

Estos son los prerequisitos técnicos sin los cuales los dashboards descritos no pueden mostrar datos reales:

| Prerequisito | Gap en v3 | Sin esto, estos paneles están vacíos |
|-------------|-----------|--------------------------------------|
| Bridge `IVRankCalculator` → `atlas_options_iv_rank_value` Prometheus | R-12 | S-01, S-02, S-09, H-09, S-05 (sentinela) |
| Bridge `OptionsFlowProvider` → Prometheus (gamma_bias, iv_current, pcr) | R-15 | S-05, S-06, Scanner panel superior |
| `OptionsPaperJournal` reader (método `read_events()`) | R-10 | P-08, P-09, columnas nuevas OptionStrat |
| Invocar `log_close_decision` desde CLIs | Sprint 1 v3 | P-06, columna motivo cierre OptionStrat |
| Integrar `VisualSignalAdapter` en orchestrador | R-05 | S-10 (parcialmente) |
| Emitir `atlas_options_session_go_nogo` desde orchestrador | Nuevo | H-01, bloque `8791/ui`, bloque Code Quant, AQP |
| Métricas de performance paper (WR, PF, equity, drawdown) | Nuevo | P-01, P-02, P-03, P-04, P-11, AQP sección |

El orden de implementación recomendado es exactamente el de la tabla anterior — resolver primero los bridges más simples (IV Rank, options flow → Prometheus) y el reader del journal, que son los que desbloquean el mayor número de paneles.

---

*Documento generado como arquitectura de observabilidad. Sin modificación de código.*  
*Base de referencia: `options_audit_v3_full.md` (ATLAS Options Engine — Auditoría v3, 19 abril 2026)*  
*El sistema está en paper mode. Esta arquitectura está preparada para escalar a live sin rediseño.*
