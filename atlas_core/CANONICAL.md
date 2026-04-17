# atlas_core — núcleo canónico

Este directorio (`atlas_core/` en la raíz del repositorio) es la **implementación canónica** del núcleo ATLAS Core: autonomía, brain y adaptadores compartidos con otros componentes (p. ej. Atlas Code-Quant vía shim).

## Qué entra en “core”

| Paquete | Rol |
|---------|-----|
| `atlas_core.autonomy` | Orquestación, registro de módulos, políticas de autonomía. |
| `atlas_core.brain` | Estado, buses, arbitraje y núcleo cognitivo. |
| `atlas_core.adapters` | Adaptadores hacia cuantificación, robot, visión, etc. |
| `atlas_core.runtime` | Bootstrap, heartbeat, bucles de evento. |

La lista exacta de módulos que deben importarse sin error en un entorno de referencia está en **`MANIFEST.json`** (misma carpeta).

## Relación con Atlas Code-Quant

El paquete `atlas_code_quant.atlas_core` es un **shim**: carga el `atlas_core` de esta raíz mediante `importlib` para que el código Quant pueda hacer `import atlas_core` sin duplicar el árbol. Ver `atlas_code_quant/atlas_core/__init__.py`.

## Reproducibilidad (nivel scaffolding)

1. Python 3.11+ recomendado (alineado con el resto del monorepo).
2. Raíz del repo en `PYTHONPATH` (o ejecutar tests/scripts desde la raíz `ATLAS_PUSH`).
3. Script de verificación:  
   `python atlas_code_quant/scripts/verify_atlas_core_manifest.py`  
   Comprueba que cada entrada del `MANIFEST.json` importa correctamente (código de salida ≠ 0 si falla).

No sustituye un entorno Docker completo ni fija revisiones de todas las dependencias transitivas del monorepo; es el **mínimo** para validar que el núcleo declarado es importable y localizable.

## Versionado

La versión lógica del núcleo sigue al **VCS** (commit de Git). No se genera SHA automáticamente en esta fase; los releases pueden anotar el commit en notas de versión o ampliar este documento con un proceso CI explícito.
