"""atlas_push.intents — enrutamiento de intents textuales.

Expone:

- `IntentRouter`: envoltorio tipado sobre `modules.command_router.handle`.
- `IntentResult`: dataclass inmutable con el resultado de un intent.
- `Kind`: alias `Literal` con los 12 tipos de intent soportados + `"empty"`.
"""

from atlas_push.intents.types import IntentResult, Kind
from atlas_push.intents.intent_router import IntentRouter

__all__ = ["IntentRouter", "IntentResult", "Kind"]
