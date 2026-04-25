\# Atlas Scanner – Contratos de datos (S0)



Este documento fija los contratos de datos del módulo `atlas\_scanner` como parte del ecosistema Atlas Core.  

Su objetivo es que cualquier implementación futura del scanner (fases S1–S3, providers reales, conectores externos) pueda evolucionar sin romper la API de integración con el resto de Atlas.



Los contratos aquí definidos se consideran \*\*estables a partir de S0\*\*: los nombres de tipos y campos solo podrán cambiar mediante una migración explícita.



\---



\## 1. Principios de diseño



\- \*\*Stateless y determinista\*\*: dado un `ScanSnapshot` de entrada, el scanner debe producir siempre el mismo `ScannerRunResult`, sin depender de estado externo (bases de datos, logs, caches, etc.).

\- \*\*Contratos pequeños y explícitos\*\*: preferimos estructuras concisas que se puedan versionar fácilmente, en vez de grafos de objetos complejos.

\- \*\*Formato neutral\*\*: los contratos se describen en términos de tipos primitivos (string, number, boolean, enum, array, object) para que puedan mapearse igual de bien a Python, TypeScript u otros lenguajes usados en Atlas.

\- \*\*Campos extensibles y versionados\*\*: cuando sea razonable, se reserva un campo `meta` o `extra` para extensiones no rompientes.



\---



\## 2. `ScanSnapshot`



Representa la \*\*foto\*\* del universo de activos opcionales que el scanner va a evaluar en una ejecución S0.



La idea es que S0 trabaje sobre snapshots generados por otros módulos (por ejemplo, una foto del universo SPY/NQ/Índices), sin depender todavía de feeds de datos en vivo.



```text

ScanSnapshot

\- snapshot\_id: string            # identificador único de la ejecución de scan (UUID)

\- created\_at: string             # ISO 8601, en UTC

\- universe\_name: string          # nombre lógico del universo (ej: "SPY\_NQ\_INTRADAY")

\- symbols: SymbolSnapshot\[]      # lista de símbolos a evaluar

\- config\_version: string         # versión del scoring/configuración usada

\- meta: object                   # extensiones futuras (tags, origen del snapshot, etc.)

```



\### 2.1 `SymbolSnapshot`



Describe la información mínima de un activo opcional a evaluar en S0.



```text

SymbolSnapshot

\- symbol: string                 # ticker subyacente (ej: "SPY")

\- asset\_type: string             # ej: "equity", "etf", "future"

\- base\_currency: string          # ej: "USD"

\- ref\_price: number              # precio de referencia usado para S0 (close, last, mid…)

\- volatility\_lookback: number    # volatilidad estimada (horizonte fijo para S0)

\- liquidity\_score: number        # proxy de liquidez (0–1 o escala definida en config)

\- meta: object                   # extensiones (sector, industry, tags propios de Atlas, etc.)

```



\---



\## 3. `CandidateOpportunity`



Representa un \*\*candidato\*\* generado por el scanner para ser enviado al pipeline de decisión de Atlas (DecisionEngine).



En S0 no hay todavía contratos con brokers ni providers externos; el foco está en normalizar el candidato de forma que futuras fases puedan rellenar más campos sin romper la forma básica.



```text

CandidateOpportunity

\- candidate\_id: string           # UUID único del candidato dentro de un ScanSnapshot

\- snapshot\_id: string            # referencia al ScanSnapshot que lo originó

\- symbol: string                 # ticker del subyacente (ej: "SPY")

\- direction: "long" | "short"    # dirección de la idea

\- thesis: string                 # explicación breve / etiqueta (ej: "mean\_reversion\_intraday")

\- score: number                  # score numérico normalizado (ej: 0–100 o 0–1)

\- score\_breakdown: ScoreBreakdown

\- time\_horizon\_minutes: number   # horizonte temporal objetivo (ej: 30, 240, 1440…)

\- max\_risk\_pct: number           # % de riesgo máximo relativo al capital asignado

\- expected\_rr: number            # ratio riesgo/beneficio esperado (ej: 2.5 => 2.5:1)

\- tags: string\[]                 # etiquetas libres (ej: \["intraday","scanner\_s0","etf"])

\- meta: object                   # extensiones futuras (IDs de señales, features, etc.)

```



\### 3.1 `ScoreBreakdown`



Permite entender de dónde sale el `score` global del candidato.



```text

ScoreBreakdown

\- raw\_score: number              # score sin normalizar (output directo del modelo/reglas)

\- normalized\_score: number       # score reescalado a la métrica estándar del scanner

\- components: ScoreComponent\[]   # desglose opcional por factor

```



```text

ScoreComponent

\- name: string                   # nombre del factor (ej: "momentum\_5m")

\- contribution: number           # contribución al score final (positiva o negativa)

\- weight: number                 # peso relativo del factor dentro del modelo

```



\---



\## 4. `ScannerRunResult`



Es la salida principal de una ejecución del módulo `atlas\_scanner`: dado un `ScanSnapshot` de entrada, produce un conjunto de candidatos y métricas agregadas de la ejecución.



```text

ScannerRunResult

\- snapshot\_id: string                # eco del snapshot de entrada

\- started\_at: string                 # ISO 8601 UTC

\- finished\_at: string                # ISO 8601 UTC

\- candidates: CandidateOpportunity\[] # lista de candidatos generados

\- metrics: ScannerRunMetrics         # métricas de la ejecución

\- meta: object                       # extensiones futuras (warnings, flags, etc.)

```



\### 4.1 `ScannerRunMetrics`



```text

ScannerRunMetrics

\- total\_symbols: number              # nº de símbolos en el snapshot

\- evaluated\_symbols: number          # nº de símbolos realmente evaluados

\- generated\_candidates: number       # nº total de candidatos generados

\- avg\_score: number                  # score medio de los candidatos

\- p95\_score: number                  # percentil 95 del score (para ver colas)

\- error\_count: number                # nº de símbolos que fallaron por algún motivo

\- warnings: string\[]                 # mensajes de warning de la ejecución

```



\---



\## 5. Campos reservados y evolución futura



Para facilitar la evolución de S0 a fases posteriores:



\- Se reserva `meta: object` en las estructuras principales (`ScanSnapshot`, `SymbolSnapshot`, `CandidateOpportunity`, `ScannerRunResult`) para adjuntar información específica de providers, brokers o modelos sin tocar el contrato base.

\- Cualquier cambio \*\*rompiente\*\* (renombrar campos, cambiar tipos) deberá hacerse vía:

&#x20; - incremento explícito de `config\_version`, y

&#x20; - documentación en `ATLAS\_SCANNER\_ARCHITECTURE.md` y `PLAN\_SCANNER\_S0.md`.



Mientras estemos en S0, los cambios se limitarán a añadir campos opcionales o detalles dentro de `meta` y `score\_breakdown`, manteniendo estables los nombres y tipos definidos en este documento.

