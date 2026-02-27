"""Ollama Provider - Local LLM inference."""
import os
import requests
from typing import Optional, Dict, Any


class OllamaProvider:
    """Ollama local LLM provider with fallback handling."""
    
    def __init__(self):
        self.base_url = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434")
        self.model = os.getenv("OLLAMA_MODEL", "deepseek-r1:latest")
        self.timeout = 30
    
    def is_available(self) -> bool:
        """Check if Ollama is running and responsive."""
        try:
            response = requests.get(f"{self.base_url}/api/tags", timeout=2)
            return response.status_code == 200
        except Exception:
            return False
    
    def generate(self, prompt: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate completion using Ollama.
        
        Args:
            prompt: The prompt to send to the model
            context: Optional context dictionary
            
        Returns:
            Dict with 'ok', 'response', 'provider', 'error' fields
        """
        if not self.is_available():
            return {
                "ok": False,
                "error": "Ollama not available",
                "provider": "ollama",
                "response": None
            }
        
        try:
            payload = {
                "model": self.model,
                "prompt": prompt,
                "stream": False
            }
            
            response = requests.post(
                f"{self.base_url}/api/generate",
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                data = response.json()
                return {
                    "ok": True,
                    "response": data.get("response", ""),
                    "provider": "ollama",
                    "model": self.model,
                    "error": None
                }
            else:
                return {
                    "ok": False,
                    "error": f"Ollama returned status {response.status_code}",
                    "provider": "ollama",
                    "response": None
                }
                
        except Exception as e:
            return {
                "ok": False,
                "error": str(e),
                "provider": "ollama",
                "response": None
            }
