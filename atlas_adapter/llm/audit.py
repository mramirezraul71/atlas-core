"""Audit logging for LLM operations in JSONL format."""
import os
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Any


class AuditLogger:
    """JSONL audit logger for LLM operations."""
    
    def __init__(self):
        self.audit_dir = Path(os.getenv("ATLAS_AUDIT_DIR", r"C:\ATLAS\logs"))
        self.audit_file = self.audit_dir / "llm_audit.jsonl"
        self._ensure_dir()
    
    def _ensure_dir(self):
        """Ensure audit directory exists."""
        try:
            self.audit_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            # Fallback to local logs
            self.audit_dir = Path("logs")
            self.audit_dir.mkdir(parents=True, exist_ok=True)
            self.audit_file = self.audit_dir / "llm_audit.jsonl"
    
    def log(self, event_type: str, data: Dict[str, Any]):
        """
        Log an event to the audit file.
        
        Args:
            event_type: Type of event (e.g., 'generate', 'fallback', 'error')
            data: Event data dictionary
        """
        try:
            entry = {
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "event_type": event_type,
                **data
            }
            
            with open(self.audit_file, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
                
        except Exception as e:
            # Silent fail - don't break the application
            print(f"Audit log error: {e}")
    
    def log_generation(self, provider: str, model: str, prompt: str, response: str, success: bool, error: str = None):
        """Log a generation event."""
        self.log("generate", {
            "provider": provider,
            "model": model,
            "prompt_length": len(prompt),
            "response_length": len(response) if response else 0,
            "success": success,
            "error": error
        })
    
    def log_fallback(self, from_provider: str, to_provider: str, reason: str):
        """Log a fallback event."""
        self.log("fallback", {
            "from_provider": from_provider,
            "to_provider": to_provider,
            "reason": reason
        })
