# AGENTS.md — Biblioteca de Conocimiento ATLAS-Quant

Este módulo es la biblioteca académica consultable del sistema ATLAS-Quant.
Contiene referencias curadas sobre comportamiento de precios, patrones de mercado
y metodologías cuantitativas, organizadas para consulta programática y humana.

---

## Reglas para agentes que amplíen esta biblioteca

1. **Priorizar fuentes académicas**: Journal of Finance, JFE, RFS, NBER, SSRN, Wiley, Princeton.
2. **Resumir con palabras propias**: no copiar texto protegido.
3. **Mantener trazabilidad**: cada entrada debe tener id, autores, año, fuente y URL verificable.
4. **Señalar incertidumbre**: `confidence_level` = high / medium / low / contested.
5. **Diferenciar**: correlación, patrón visual, relación estadística y causalidad.
6. **Si hay desacuerdo en la literatura**: indicarlo en `limitations` Y en `tags` con `contested`.
7. **Al agregar una fuente nueva**:
   - Añadir entrada en `sources_catalog.py → SOURCES`
   - Actualizar `SCANNER_METHOD_TO_SOURCES` si aplica
   - Actualizar `atlas_index.md`
   - Crear ficha en `fichas/` (opcional pero recomendado)

---

## Estructura del módulo

```
knowledge/
├── __init__.py              — exports: KnowledgeBase, get_knowledge_base
├── knowledge_base.py        — motor de consulta (advisory_context, query por método/timeframe/tema)
├── sources_catalog.py       — catálogo curado (SOURCES list + índices)
├── AGENTS.md                — este archivo
├── atlas_index.md           — índice navegable por categoría y temporalidad
└── fichas/                  — fichas individuales JSON por referencia (opcional)
```

---

## Esquema de campos obligatorios para cada referencia

```python
{
    "id":               str,   # snake_case único: autor_año_keyword
    "title":            str,   # título exacto de la obra
    "authors":          list,  # lista de autores
    "year":             int,   # año de publicación
    "source":           str,   # revista, editorial, working paper
    "url":              str,   # URL directa o DOI (vacío si no disponible)
    "category":         str,   # CAT1-CAT5
    "topic":            str,   # tema principal (snake_case)
    "subtopics":        list,  # lista de subtemas
    "timeframes":       list,  # ["intraday", "short", "swing", "medium", "long", "all"]
    "markets":          list,  # ["equity", "options", "futures", "fx", "crypto"]
    "method":           str,   # metodología del paper
    "key_findings":     str,   # hallazgos principales (max 500 chars)
    "limitations":      str,   # limitaciones conocidas (max 300 chars)
    "atlas_utility":    str,   # cómo usar en ATLAS-Quant (max 300 chars)
    "confidence_level": str,   # "high" | "medium" | "low" | "contested"
    "scanner_methods":  list,  # métodos del scanner ATLAS que esta fuente respalda
    "tags":             list,  # keywords para búsqueda
}
```

---

## Categorías del catálogo

| Cat | Descripción |
|-----|-------------|
| CAT1 | Teoría base: eficiencia, caminata aleatoria, microestructura |
| CAT2 | Fenómenos de precio: momentum, reversión, tendencia |
| CAT3 | Patrones visuales: chartismo, Elliott, H&S, tops/bottoms |
| CAT4 | Modelos explicativos: conductual, risk-based, opciones |
| CAT5 | Metodología: series de tiempo, ML, validación, ejecución |

---

## Temporalidades reconocidas

| Clave | Descripción |
|-------|-------------|
| `intraday` | < 1 día — microestructura, order flow dominan |
| `short` | 1–5 días — reversión de corto plazo, bid-ask bounce |
| `swing` | 5–60 días — momentum, continuación de tendencia |
| `medium` | 60–252 días — momentum cross-seccional, ciclos |
| `long` | > 252 días — reversión a la media, fundamentales |
| `all` | Independiente de temporalidad |

---

## Integración con el sistema ATLAS

La `KnowledgeBase` se integra en:

1. **_auto_cycle_loop** (`api/main.py`): `advisory_context()` se adjunta al resultado de cada
   ciclo cuando `action == "submit"`. Es no-bloqueante — no puede vetar órdenes.

2. **Endpoints REST**:
   - `GET /knowledge/query` — consulta por método, timeframe y tema
   - `GET /knowledge/sources` — lista todo el catálogo
   - `GET /knowledge/summary` — estadísticas de la biblioteca

3. **Scorecard** (`trading_implementation_scorecard.py`): el `academic_support_score` del
   advisory context informa la calidad de respaldo de cada señal del scanner.

---

## Fuentes prioritarias para ampliar

- **Journal of Finance** — `https://onlinelibrary.wiley.com/journal/15406261`
- **Journal of Financial Economics** — `https://www.sciencedirect.com/journal/journal-of-financial-economics`
- **Review of Financial Studies** — `https://academic.oup.com/rfs`
- **NBER Working Papers** — `https://www.nber.org/papers`
- **SSRN Finance** — `https://ssrn.com/en/index.cfm/finance/`
- **AQR Insights** — `https://www.aqr.com/Insights/Research`
- **Man AHL Research** — `https://www.man.com/ahl`

---

## Criterio de calidad para nuevas entradas

Una fuente se considera lista cuando:
- [ ] Tiene `id` único y `url` verificable
- [ ] `key_findings` captura el aporte principal en ≤ 500 chars
- [ ] `atlas_utility` explica concretamente cómo usar en ATLAS en ≤ 300 chars
- [ ] `confidence_level` está justificado en `limitations`
- [ ] `scanner_methods` lista los métodos del scanner que respalda (puede estar vacío)
- [ ] `tags` incluye al menos 3 keywords relevantes
