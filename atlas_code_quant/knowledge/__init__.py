"""Knowledge module — biblioteca académica consultable para toma de decisiones ATLAS-Quant.

Estructura:
    sources_catalog.py  — catálogo curado de 30+ referencias con metadatos normalizados
    knowledge_base.py   — motor de consulta: query por tema, temporalidad, método scanner
    fichas/             — fichas JSON individuales por referencia (generadas desde el catálogo)
    AGENTS.md           — instrucciones para agentes que ampíen la biblioteca
    atlas_index.md      — índice humano navegable

Uso rápido:
    from atlas_code_quant.knowledge import get_knowledge_base
    kb = get_knowledge_base()
    ctx = kb.advisory_context(method="trend_ema_stack", timeframe="1d")
"""
from atlas_code_quant.knowledge.knowledge_base import KnowledgeBase, get_knowledge_base

__all__ = ["KnowledgeBase", "get_knowledge_base"]
