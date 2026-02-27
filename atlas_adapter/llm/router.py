"""Intelligent LLM router with fallback support."""
from typing import Dict, Any, Optional
from .providers.ollama import OllamaProvider
from .providers.openai import OpenAIProvider
from .policy import Policy
from .audit import AuditLogger


class LLMRouter:
    """Intelligent router for LLM requests with automatic fallback."""
    
    def __init__(self):
        self.ollama = OllamaProvider()
        self.openai = OpenAIProvider()
        self.policy = Policy()
        self.audit = AuditLogger()
    
    def generate(self, prompt: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate completion with intelligent routing and fallback.
        
        Args:
            prompt: The prompt to send to the model
            context: Optional context dictionary
            
        Returns:
            Dict with 'ok', 'response', 'provider', 'fallback_used', 'error' fields
        """
        primary = self.policy.get_primary_provider()
        
        # Try primary provider
        result = self._try_provider(primary, prompt, context)
        
        if result["ok"]:
            self.audit.log_generation(
                provider=result["provider"],
                model=result.get("model", "unknown"),
                prompt=prompt,
                response=result["response"],
                success=True
            )
            return {
                **result,
                "fallback_used": False
            }
        
        # Primary failed, try fallback
        fallback = self.policy.get_fallback_provider(primary)
        
        if not fallback:
            # No fallback available
            self.audit.log_generation(
                provider=primary,
                model="unknown",
                prompt=prompt,
                response="",
                success=False,
                error=result.get("error", "Unknown error")
            )
            return {
                **result,
                "fallback_used": False
            }
        
        # Log fallback attempt
        self.audit.log_fallback(
            from_provider=primary,
            to_provider=fallback,
            reason=result.get("error", "Unknown error")
        )
        
        # Try fallback provider
        fallback_result = self._try_provider(fallback, prompt, context)
        
        self.audit.log_generation(
            provider=fallback_result["provider"],
            model=fallback_result.get("model", "unknown"),
            prompt=prompt,
            response=fallback_result.get("response", ""),
            success=fallback_result["ok"],
            error=fallback_result.get("error")
        )
        
        return {
            **fallback_result,
            "fallback_used": True,
            "primary_error": result.get("error")
        }
    
    def _try_provider(self, provider_name: str, prompt: str, context: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """Try a specific provider."""
        if provider_name == "ollama":
            return self.ollama.generate(prompt, context)
        elif provider_name == "openai":
            return self.openai.generate(prompt, context)
        else:
            return {
                "ok": False,
                "error": f"Unknown provider: {provider_name}",
                "provider": provider_name,
                "response": None
            }
