"""Policy engine for LLM routing decisions."""
import os
from typing import Literal


LLMMode = Literal["local", "cloud", "hybrid"]


class Policy:
    """Policy engine for determining LLM routing behavior."""
    
    def __init__(self):
        self.mode: LLMMode = os.getenv("ATLAS_LLM_MODE", "hybrid")
        self.default_provider = os.getenv("ATLAS_LLM_DEFAULT", "ollama")
        self.strict_offline = os.getenv("ATLAS_LLM_STRICT_OFFLINE", "0") == "1"
    
    def should_use_local(self) -> bool:
        """Check if local provider should be used."""
        return self.mode in ("local", "hybrid")
    
    def should_use_cloud(self) -> bool:
        """Check if cloud provider should be used."""
        if self.strict_offline:
            return False
        return self.mode in ("cloud", "hybrid")
    
    def get_primary_provider(self) -> str:
        """Get the primary provider to try first."""
        if self.mode == "local":
            return "ollama"
        elif self.mode == "cloud":
            return "openai"
        else:  # hybrid
            return self.default_provider
    
    def get_fallback_provider(self, failed_provider: str) -> str:
        """Get fallback provider when primary fails."""
        if self.strict_offline:
            return None
        
        if failed_provider == "ollama":
            return "openai" if self.should_use_cloud() else None
        elif failed_provider == "openai":
            return "ollama" if self.should_use_local() else None
        
        return None
