\# Atlas Scanner – Fases de evolución



Este documento define las fases de evolución del módulo `atlas\_scanner` a partir de S0.  

El objetivo es tener un roadmap claro y realista, alineado con el resto de Atlas Core y con los módulos ya existentes (`atlas\_push`, `DecisionEngine`, etc.).



\---



\## S0 – Scaffold y contratos



Estado actual.



\- Crear el módulo `atlas\_scanner` como hermano de `atlas\_push` dentro de `atlas-core`.

\- Definir y documentar:

&#x20; - `ScanSnapshot`, `SymbolSnapshot`

&#x20; - `CandidateOpportunity`, `ScoreBreakdown`, `ScannerRunResult`

\- Describir la arquitectura interna:

&#x20; - Pipeline conceptual `ScanSnapshot -> ScannerRunResult`

&#x20; - Límite stateless, sin side-effects persistentes

\- Introducir los primeros tests basados en snapshots sintéticos.



\*\*Resultado de S0\*\*:  

Existe un módulo `atlas\_scanner` bien definido, con contratos estables y documentación suficiente para que futuras fases puedan implementarse sin rediseñar la interfaz.



\---



\## S1 – Primer scanner funcional con datos offline



Objetivo: pasar de un scaffold documental a un scanner que genere candidatos \*\*de verdad\*\*, aunque sea con datos offline o fixtures.



\- Implementar un pipeline mínimo:

&#x20; - Carga de `ScanSnapshot` desde archivos (JSON/fixtures).

&#x20; - Validación y normalización básica.

&#x20; - Estrategia simple de scoring (por ejemplo, reglas deterministas sobre volatilidad / liquidez).

\- Integrar con el `DecisionEngine` en modo “read-only”:

&#x20; - El scanner produce `ScannerRunResult`.

&#x20; - El engine consume resultados y decide qué haría, sin ejecutar órdenes reales.

\- Añadir métricas básicas:

&#x20; - nº de símbolos evaluados, nº de candidatos, distribución de scores.

\- Añadir tests deterministas sobre casos representativos.



\*\*Resultado de S1\*\*:  

El scanner puede ejecutarse de punta a punta con datos offline y producir candidatos que el `DecisionEngine` sabe leer, aunque todavía no estén conectados a feeds en tiempo real.



\---



\## S2 – Integración con providers y signals reales



Objetivo: conectar el scanner con fuentes de datos reales y enriquecer la lógica de scoring.



\- Añadir capa de adquisición de datos:

&#x20; - Integración con uno o varios providers (por ejemplo, data feed interno, CSVs generados por Atlas, etc.).

&#x20; - Generación programática de `ScanSnapshot` a partir de esos datos.

\- Ampliar el sistema de features:

&#x20; - Indicadores técnicos sencillos (ej. momentum, rango intradía, volatilidad reciente).

&#x20; - Posible integración de señales generadas por otros módulos de Atlas.

\- Evolucionar el scoring:

&#x20; - De reglas deterministas simples a combinaciones multi-factor.

&#x20; - Mejor uso de `ScoreBreakdown` para trazabilidad.

\- Añadir más métricas:

&#x20; - Ratio candidatos/símbolos.

&#x20; - Estadísticas por tipo de activo, por universo, por franja horaria.



\*\*Resultado de S2\*\*:  

El scanner trabaja con datos vivos o actualizados, genera candidatos más ricos y empieza a ser una fuente útil y regular para el motor de decisiones.



\---



\## S3 – Conectores específicos y refinamiento



Objetivo: convertir el scanner en una pieza de infraestructura sólida, conectada con el resto del ecosistema Atlas.



\- Conectores específicos:

&#x20; - Integración con módulos como Trader, OpenBB, Vision o equivalentes internos.

&#x20; - Uso de estos conectores para:

&#x20;   - alimentar `ScanSnapshot` con universos más inteligentes, o

&#x20;   - consumir `ScannerRunResult` y derivar acciones.

\- Refinamiento del pipeline:

&#x20; - Mejor gestión de errores y warnings en `ScannerRunMetrics`.

&#x20; - Opcionalmente, soporte para distintos “modos” de scanner (intraday, swing, research).

\- Gobernanza de configuración:

&#x20; - Gestión explícita de versiones de `config\_version`.

&#x20; - Capacidad de comparar resultados entre versiones de configuración.



\*\*Resultado de S3\*\*:  

`atlas\_scanner` es un módulo estable, integrado, con contratos respetados, configuraciones versionadas y suficiente observabilidad como para mantenerlo y evolucionarlo en producción.



\---



\## Más allá de S3



A partir de S3, las líneas de evolución probables son:



\- Estrategias más complejas (modelos ML, clustering de oportunidades, etc.).

\- Orquestación de múltiples scanners especializados (por universo, por horizonte, por tipo de señal).

\- Feedback loop con resultados reales (aprendizaje a partir de trades ejecutados).



Estos temas se documentarán en planes específicos posteriores, una vez consolidada la base definida en S0–S3.

