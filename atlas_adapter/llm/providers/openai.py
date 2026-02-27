"""OpenAI Provider - Cloud LLM inference."""
import os
from typing import Optional, Dict, Any


class OpenAIProvider:
    """OpenAI cloud LLM provider with API key handling."""
    
    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY", "")
        self.model = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
        self.timeout = 30
    
    def is_available(self) -> bool:
        """Check if OpenAI API key is configured."""
        return bool(self.api_key and self.api_key.startswith("sk-"))
    
    def generate(self, prompt: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate completion using OpenAI API.
        
        Args:
            prompt: The prompt to send to the model
            context: Optional context dictionary
            
        Returns:
            Dict with 'ok', 'response', 'provider', 'error' fields
        """
        if not self.is_available():
            return {
                "ok": False,
                "error": "OpenAI API key not configured",
                "provider": "openai",
                "response": None
            }
        
        try:
            from openai import OpenAI
            
            client = OpenAI(api_key=self.api_key, timeout=self.timeout)
            
            messages = [{"role": "user", "content": prompt}]
            
            response = client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )
            
            content = response.choices[0].message.content
            
            return {
                "ok": True,
                "response": content,
                "provider": "openai",
                "model": self.model,
                "error": None
            }
                
        except Exception as e:
            return {
                "ok": False,
                "error": str(e),
                "provider": "openai",
                "response": None
            }
