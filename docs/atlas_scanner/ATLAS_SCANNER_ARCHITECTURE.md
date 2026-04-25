\# Atlas Scanner – Arquitectura S0



Este documento describe la arquitectura del módulo `atlas\_scanner` en la fase S0:  

un componente separado dentro de Atlas Core, capaz de evaluar un universo de activos opcionales y producir `ScannerRunResult` normalizados para el pipeline de decisión.



\---



\## 1. Objetivo de la arquitectura S0



\- Definir un módulo \*\*separado pero hermano\*\* de `atlas\_push`, viviendo en el mismo repositorio `atlas-core`.

\- Proporcionar un punto claro donde iterar futuras fases (features reales, conectores Trader/OpenBB/Vision, etc.) sin tocar el núcleo de Atlas.

\- Mantener una frontera limpia con el `DecisionEngine`: el scanner produce `ScannerRunResult`; el engine decide qué hacer con ellos.



\---



\## 2. Posicionamiento en Atlas Core



A alto nivel:



```text

\[ Datos / Providers / Snapshots ]  --->  \[ atlas\_scanner ]  --->  \[ DecisionEngine / atlas\_push ]

&#x20;                        ScanSnapshot           ScannerRunResult

```



\- El scanner \*\*no\*\* habla directamente con brokers, colas ni bases de datos.

\- Trabaja sobre `ScanSnapshot` generados por otros módulos (o por fixtures en S0).

\- Su responsabilidad es puramente analítica: transformar `ScanSnapshot` → `ScannerRunResult`.



\---



\## 3. Límite de módulo y dependencias



\### 3.1 Módulo separado, mismo repo



\- El código de `atlas\_scanner` vive dentro de `atlas-core`, como módulo hermano de `atlas\_push`.

\- No se mezcla código de scanner dentro de `atlas\_push/engine` ni en `atlas\_push/intents`.

\- El scanner puede depender de utilidades compartidas (logging, tipos básicos, helpers), pero \*\*no\*\* debe introducir dependencias circulares con `atlas\_push`.



\### 3.2 Stateless por diseño



\- Cada ejecución del scanner es \*\*stateless\*\*: recibe un `ScanSnapshot` autocontenido y produce un `ScannerRunResult`.

\- El módulo no guarda estado duradero (no escribe a bases de datos, no cachea entre ejecuciones).

\- Cualquier necesidad futura de estado (caches de features, histórico, etc.) se implementará en capas externas que preparan el `ScanSnapshot`.



\---



\## 4. Flujo interno de una ejecución



El pipeline interno de S0, de forma conceptual:



```text

1\) Input: ScanSnapshot

2\) Normalización / Validación

3\) Generación de features (si aplica en S0)

4\) Estrategias de scoring / reglas

5\) Construcción de CandidateOpportunity

6\) Agregado a ScannerRunResult

7\) Métricas de ejecución

```



\### 4.1 Pasos



1\. \*\*Input\*\*  

&#x20;  - Se recibe un `ScanSnapshot` válido (ver contratos) desde el caller.

&#x20;  - S0 asume que el snapshot ya contiene el universo y precios de referencia.



2\. \*\*Normalización / Validación\*\*  

&#x20;  - Validación mínima de tipos y presencia de campos obligatorios.

&#x20;  - Posible normalización de escalas (por ejemplo, `liquidity\_score` a \[0,1]).



3\. \*\*Features\*\*  

&#x20;  - En S0 puede ser una capa muy fina (o incluso un stub) que prepara inputs para el scoring.

&#x20;  - En fases posteriores se puede enriquecer con indicadores técnicos, datos fundamentales, etc.



4\. \*\*Scoring / Reglas\*\*  

&#x20;  - Aplicación de reglas o modelos que producen un `raw\_score` por símbolo o por candidato.

&#x20;  - Transformación a `normalized\_score` según `config\_version`.



5\. \*\*Construcción de candidatos\*\*  

&#x20;  - Por cada símbolo/caso que supera umbrales, se construye un `CandidateOpportunity` con su `ScoreBreakdown`.

&#x20;  - Se añaden `tags` y meta-información relevante para el DecisionEngine.



6\. \*\*Agregado\*\*  

&#x20;  - Se agregan todos los `CandidateOpportunity` en un `ScannerRunResult`.

&#x20;  - Se rellenan campos de timestamps (`started\_at`, `finished\_at`) y métricas (`ScannerRunMetrics`).



7\. \*\*Salida\*\*  

&#x20;  - Se devuelve el `ScannerRunResult` al caller.

&#x20;  - No se realizan side-effects persistentes dentro del módulo.



\---



\## 5. Configuración y versiones



\- La configuración de scoring y filtros vive en una estructura centralizada (ej. `SCORING\_CONFIG`), versionada mediante `config\_version` en `ScanSnapshot`.

\- Cambios de configuración \*\*no\*\* cambian los nombres de campos de los contratos, solo su interpretación.

\- Cambios rompientes de contratos se documentarán en este archivo y en `PLAN\_SCANNER\_S0.md`.



\---



\## 6. Evolución más allá de S0



S0 solo fija la arquitectura mínima. Las futuras fases añaden capacidades respetando estos límites:



\- \*\*S1\*\*: integración con primeros providers de datos, generación de `ScanSnapshot` reales.

\- \*\*S2\*\*: estrategias más ricas (multi-factor), más features, métricas avanzadas.

\- \*\*S3\*\*: conectores específicos (Trader, OpenBB, Vision) que alimentan snapshots o consumen resultados, sin acoplarse al interior del scanner.



Mientras tanto, S0 se centra en tener un módulo:



\- Claro en responsabilidades.

\- Estable en contratos.

\- Fácil de testear mediante snapshots sintéticos y pruebas deterministas.

