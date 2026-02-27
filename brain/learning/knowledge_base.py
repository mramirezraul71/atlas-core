"""
ATLAS Knowledge Base
Repositorio central de conocimiento del tutor Opus para Continual Learning.
Maneja almacenamiento y recuperación de conocimientos consolidados.
"""
import os
import json
import logging
from typing import List, Dict, Any, Optional
from datetime import datetime

logger = logging.getLogger(__name__)

class KnowledgeBase:
    def __init__(self, storage_dir: str = "C:/ATLAS_PUSH/data/knowledge"):
        self.storage_dir = storage_dir
        self.file_path = os.path.join(self.storage_dir, "opus_knowledge.json")
        self._ensure_storage()
        self.knowledge = self._load()

    def _ensure_storage(self):
        os.makedirs(self.storage_dir, exist_ok=True)
        if not os.path.exists(self.file_path):
            with open(self.file_path, "w", encoding="utf-8") as f:
                json.dump({"concepts": [], "rules": [], "experiences": []}, f, indent=2)

    def _load(self) -> Dict[str, List[Any]]:
        try:
            with open(self.file_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Error loading knowledge base: {e}")
            return {"concepts": [], "rules": [], "experiences": []}

    def _save(self):
        try:
            with open(self.file_path, "w", encoding="utf-8") as f:
                json.dump(self.knowledge, f, indent=2)
        except Exception as e:
            logger.error(f"Error saving knowledge base: {e}")

    def add_concept(self, name: str, description: str, context: str = ""):
        self.knowledge.setdefault("concepts", []).append({
            "name": name,
            "description": description,
            "context": context,
            "added_at": datetime.now().isoformat()
        })
        self._save()

    def add_rule(self, rule: str, category: str = "general"):
        self.knowledge.setdefault("rules", []).append({
            "rule": rule,
            "category": category,
            "added_at": datetime.now().isoformat()
        })
        self._save()

    def get_all_concepts(self) -> List[Dict[str, Any]]:
        return self.knowledge.get("concepts", [])

    def search(self, query: str) -> Dict[str, List[Any]]:
        query = query.lower()
        results = {"concepts": [], "rules": []}
        
        for c in self.knowledge.get("concepts", []):
            if query in c.get("name", "").lower() or query in c.get("description", "").lower():
                results["concepts"].append(c)
                
        for r in self.knowledge.get("rules", []):
            if query in r.get("rule", "").lower():
                results["rules"].append(r)
                
        return results
