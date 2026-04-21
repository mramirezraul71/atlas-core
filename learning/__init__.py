from __future__ import annotations

import atlas_code_quant.learning as _atlas_learning

from atlas_code_quant.learning import *  # noqa: F401,F403

# Reuse the real package path so imports like `learning.adaptive_policy`
# resolve without requiring PYTHONPATH tweaks.
__path__ = _atlas_learning.__path__
