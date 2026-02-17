"""
Lóbulo Temporal Atlas: Procesamiento auditivo y comprensión del lenguaje.

Análogo biológico: Corteza auditiva + Área de Wernicke
- AudioProcessor: ASR, clasificación de sonidos
- LanguageUnderstanding: NLU, extracción de intenciones
"""
from .audio_processor import AudioProcessor
from .language_understanding import LanguageUnderstanding

__all__ = [
    "AudioProcessor",
    "LanguageUnderstanding",
]
