"""Local Ollama model role helpers for ATLAS Code Quant."""
from .classifier_provider import LocalClassifierProvider
from .config import LOCAL_MODEL_CONFIG, get_role_model, load_local_model_config
from .embedding_provider import LocalEmbeddingProvider
from .integrations import analyze_dashboard_screenshot, classify_journal_event, semantic_journal_search
from .ollama_client import OllamaClient, OllamaUnavailable
from .router import (
    get_classifier_provider,
    get_coder_profile,
    get_embedding_provider,
    get_ollama_client,
    get_vision_provider,
    reset_router_cache,
)
from .vision_provider import LocalVisionProvider

__all__ = [
    "LOCAL_MODEL_CONFIG",
    "OllamaClient",
    "OllamaUnavailable",
    "LocalEmbeddingProvider",
    "LocalClassifierProvider",
    "LocalVisionProvider",
    "load_local_model_config",
    "get_role_model",
    "get_ollama_client",
    "get_embedding_provider",
    "get_classifier_provider",
    "get_vision_provider",
    "get_coder_profile",
    "reset_router_cache",
    "semantic_journal_search",
    "classify_journal_event",
    "analyze_dashboard_screenshot",
]
