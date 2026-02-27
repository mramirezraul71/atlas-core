"""
Lóbulo Temporal Atlas: Procesamiento auditivo y comprensión del lenguaje.

Análogo biológico: Corteza auditiva + Área de Wernicke
- AudioProcessor: ASR, clasificación de sonidos
- LanguageUnderstanding: NLU, extracción de intenciones
- EpisodicRecall: Recuperación de memorias episódicas
"""
from .audio_processor import AudioProcessor
from .language_understanding import LanguageUnderstanding
from .episodic_recall import EpisodicRecall, RecallQuery, RecallResult, RecalledEpisode

__all__ = [
    "AudioProcessor",
    "LanguageUnderstanding",
    "EpisodicRecall",
    "RecallQuery",
    "RecallResult",
    "RecalledEpisode",
]
