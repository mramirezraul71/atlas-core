"""
ATLAS NEXUS - Supervisor Memory Integration
Conecta el Supervisor con el sistema de memoria de Atlas
"""
from __future__ import annotations

import json
import logging
from datetime import datetime
from typing import Any, Dict, List, Optional

logger = logging.getLogger("atlas.brain.supervisor_memory")


class SupervisorMemoryIntegration:
    """
    Integración entre Supervisor y el sistema de memoria de Atlas.

    Provee:
    - Memoria de auditorías y hallazgos del Supervisor
    - Contexto histórico para investigaciones
    - Persistencia de directivas y recomendaciones
    - Búsqueda de problemas similares resueltos
    """

    def __init__(self):
        self.memory_enabled = True
        self.supervisor_session_id: Optional[str] = None

        # Cargar sistemas de memoria
        self._load_memory_systems()

        logger.info("✅ Supervisor Memory Integration initialized")

    def _load_memory_systems(self):
        """Cargar sistemas de memoria con manejo de errores."""
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            self.chroma = get_chroma_memory()
            logger.info("  ✅ ChromaDB connected")
        except Exception as e:
            logger.warning(f"  ⚠️ ChromaDB not available: {e}")
            self.chroma = None

        try:
            from modules.humanoid.cortex.unified_memory import \
                get_unified_memory

            self.unified_memory = get_unified_memory()
            logger.info("  ✅ Unified Memory Cortex connected")
        except Exception as e:
            logger.warning(f"  ⚠️ Unified Memory Cortex not available: {e}")
            self.unified_memory = None

        try:
            from modules.humanoid.memory_engine.chat_memory import \
                get_chat_memory

            self.chat_memory = get_chat_memory()
            # Crear sesión específica para Supervisor
            self.supervisor_session_id = self.chat_memory.create_session(
                user_id="atlas_supervisor",
                context={"role": "supervisor", "type": "audit_memory"},
            )
            logger.info(f"  ✅ Supervisor session created: {self.supervisor_session_id}")
        except Exception as e:
            logger.warning(f"  ⚠️ Chat Memory not available: {e}")
            self.chat_memory = None

    def store_audit_finding(
        self, finding: Dict[str, Any], evidence: List[Dict[str, Any]] = None
    ):
        """
        Almacenar hallazgo de auditoría en memoria.

        Args:
            finding: Hallazgo del supervisor
            evidence: Evidencia asociada
        """
        if not self.memory_enabled or not self.chroma:
            return

        try:
            # Almacenar hallazgo principal
            finding_content = f"AUDIT FINDING: {finding.get('title', 'Unknown')}\n"
            finding_content += f"Issue: {finding.get('issue', '')}\n"
            finding_content += f"Severity: {finding.get('severity', 'medium')}\n"
            finding_content += f"Component: {finding.get('component', 'unknown')}\n"

            if evidence:
                finding_content += f"Evidence: {len(evidence)} items\n"

            self.chroma.add_memory(
                content=finding_content,
                memory_type="audit_finding",
                metadata={
                    "title": finding.get("title"),
                    "issue": finding.get("issue"),
                    "severity": finding.get("severity", "medium"),
                    "component": finding.get("component"),
                    "timestamp": datetime.now().isoformat(),
                    "evidence_count": len(evidence) if evidence else 0,
                    "session_id": self.supervisor_session_id,
                },
            )

            # Almacenar evidencia individual
            if evidence:
                for i, ev in enumerate(evidence):
                    evidence_content = (
                        f"EVIDENCE [{i+1}]: {ev.get('type', 'unknown')}\n"
                    )
                    evidence_content += f"Source: {ev.get('source', '')}\n"
                    evidence_content += f"Data: {ev.get('data', '')[:200]}...\n"

                    self.chroma.add_memory(
                        content=evidence_content,
                        memory_type="audit_evidence",
                        metadata={
                            "finding_title": finding.get("title"),
                            "evidence_type": ev.get("type"),
                            "source": ev.get("source"),
                            "timestamp": datetime.now().isoformat(),
                            "session_id": self.supervisor_session_id,
                        },
                    )

            logger.info(f"Stored audit finding: {finding.get('title')}")

        except Exception as e:
            logger.warning(f"Failed to store audit finding: {e}")

    def store_directive(self, directive: Dict[str, Any], target: str = "atlas"):
        """
        Almacenar directiva del Supervisor.

        Args:
            directive: Directiva emitida
            target: Destino (atlas, agent, owner)
        """
        if not self.memory_enabled or not self.chroma:
            return

        try:
            directive_content = f"SUPERVISOR DIRECTIVE\n"
            directive_content += f"Target: {target}\n"
            directive_content += f"Action: {directive.get('action', '')}\n"
            directive_content += f"Priority: {directive.get('priority', 'medium')}\n"
            directive_content += f"Description: {directive.get('description', '')}\n"

            if directive.get("commands"):
                directive_content += f"Commands: {len(directive['commands'])}\n"

            self.chroma.add_memory(
                content=directive_content,
                memory_type="supervisor_directive",
                metadata={
                    "target": target,
                    "action": directive.get("action"),
                    "priority": directive.get("priority", "medium"),
                    "commands_count": len(directive.get("commands", [])),
                    "timestamp": datetime.now().isoformat(),
                    "session_id": self.supervisor_session_id,
                },
            )

            logger.info(f"Stored directive for {target}: {directive.get('action')}")

        except Exception as e:
            logger.warning(f"Failed to store directive: {e}")

    def search_similar_issues(
        self, issue_description: str, component: str = None, limit: int = 10
    ) -> Dict[str, Any]:
        """
        Buscar problemas similares resueltos anteriormente.

        Args:
            issue_description: Descripción del problema actual
            component: Componente específico
            limit: Límite de resultados

        Returns:
        Problemas similares y sus soluciones
        """
        if not self.memory_enabled:
            return {"found": False, "reason": "Memory disabled"}

        results = {
            "found": False,
            "query": issue_description,
            "similar_issues": [],
            "related_directives": [],
            "evidence_patterns": [],
        }

        # 1. Buscar en Unified Memory Cortex
        if self.unified_memory:
            try:
                memory_types = ["episodic", "procedural", "semantic"]
                memories = self.unified_memory.recall(
                    query=issue_description,
                    memory_types=memory_types,
                    limit=limit,
                    min_relevance=0.4,
                )

                if memories:
                    results["found"] = True
                    results["similar_issues"] = [m.to_dict() for m in memories[:5]]
                    logger.info(
                        f"Found {len(memories)} similar issues in Unified Memory"
                    )
            except Exception as e:
                logger.warning(f"Unified Memory search failed: {e}")

        # 2. Buscar hallazgos de auditoría similares
        if self.chroma:
            try:
                # Construir query para búsqueda
                search_query = issue_description
                if component:
                    search_query += f" {component}"

                audit_results = self.chroma.search_memories(
                    query=search_query, memory_types=["audit_finding"], limit=limit
                )

                if audit_results:
                    results["found"] = True
                    results["similar_issues"].extend(
                        [
                            {
                                "source": "audit_finding",
                                "content": result["content"],
                                "metadata": result["metadata"],
                                "relevance": result["score"],
                            }
                            for result in audit_results[:5]
                        ]
                    )

                # Buscar directivas relacionadas
                directive_results = self.chroma.search_memories(
                    query=search_query,
                    memory_types=["supervisor_directive"],
                    limit=limit,
                )

                if directive_results:
                    results["related_directives"] = [
                        {
                            "content": result["content"],
                            "metadata": result["metadata"],
                            "relevance": result["score"],
                        }
                        for result in directive_results[:3]
                    ]

                logger.info(
                    f"Found {len(audit_results)} audit findings, {len(directive_results)} directives"
                )

            except Exception as e:
                logger.warning(f"ChromaDB search failed: {e}")

        return results

    def get_supervisor_history(self, limit: int = 50) -> Dict[str, Any]:
        """
        Obtener historial completo del Supervisor.

        Args:
            limit: Límite de resultados

        Returns:
            Historial de auditorías y directivas
        """
        if not self.memory_enabled or not self.chat_memory:
            return {"success": False, "error": "Memory not available"}

        try:
            # Obtener historial de sesión del Supervisor
            history = self.chat_memory.get_session_history(
                session_id=self.supervisor_session_id, limit=limit
            )

            if not history["success"]:
                return history

            # Analizar y categorizar
            findings = []
            directives = []
            evidence = []

            for msg in history["messages"]:
                content = msg["message"]

                if "AUDIT FINDING:" in content:
                    findings.append(msg)
                elif "SUPERVISOR DIRECTIVE" in content:
                    directives.append(msg)
                elif "EVIDENCE:" in content:
                    evidence.append(msg)

            return {
                "success": True,
                "session_id": self.supervisor_session_id,
                "total_messages": history["total_messages"],
                "findings": findings,
                "directives": directives,
                "evidence": evidence,
                "summaries": history["summaries"],
            }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def enhance_supervisor_analysis(
        self, objective: str, current_snapshot: Dict[str, Any]
    ) -> str:
        """
        Enhance análisis del Supervisor con contexto histórico.

        Args:
            objective: Objetivo del análisis
            current_snapshot: Snapshot actual del sistema

        Returns:
            Análisis enhanced con memoria
        """
        if not self.memory_enabled:
            return ""

        # Buscar problemas similares
        similar_issues = self.search_similar_issues(objective)

        if not similar_issues["found"]:
            return ""

        enhanced_context = "\n\n--- CONTEXTO HISTÓRICO DEL SUPERVISOR ---\n"

        # Añadir problemas similares
        if similar_issues["similar_issues"]:
            enhanced_context += "**Problemas Similares Anteriores:**\n"
            for i, issue in enumerate(similar_issues["similar_issues"][:3]):
                enhanced_context += f"{i+1}. [{issue.get('memory_type', 'unknown')}] {issue.get('content', '')[:150]}...\n"

        # Añadir directivas relacionadas
        if similar_issues["related_directives"]:
            enhanced_context += "\n**Directivas Relacionadas:**\n"
            for i, directive in enumerate(similar_issues["related_directives"][:2]):
                enhanced_context += f"{i+1}. {directive.get('content', '')[:150]}...\n"

        enhanced_context += f"\n**Total hallazgos históricos:** {len(similar_issues['similar_issues'])}\n"
        enhanced_context += f"**Total directivas relacionadas:** {len(similar_issues['related_directives'])}\n"

        return enhanced_context

    def store_investigation_result(
        self,
        objective: str,
        investigation: Dict[str, Any],
        resolution: Dict[str, Any] = None,
    ):
        """
        Almacenar resultado de investigación completa.

        Args:
            objective: Objetivo de la investigación
            investigation: Proceso y hallazgos
            resolution: Resolución aplicada
        """
        if not self.memory_enabled or not self.chroma:
            return

        try:
            # Almacenar investigación completa
            investigation_content = f"SUPERVISOR INVESTIGATION\n"
            investigation_content += f"Objective: {objective}\n"
            investigation_content += (
                f"Duration: {investigation.get('duration_seconds', 0)}s\n"
            )
            investigation_content += (
                f"Findings: {investigation.get('findings_count', 0)}\n"
            )
            investigation_content += (
                f"Evidence: {investigation.get('evidence_count', 0)}\n"
            )

            if resolution:
                investigation_content += (
                    f"Resolution: {resolution.get('status', 'unknown')}\n"
                )
                investigation_content += (
                    f"Actions taken: {len(resolution.get('actions', []))}\n"
                )

            self.chroma.add_memory(
                content=investigation_content,
                memory_type="supervisor_investigation",
                metadata={
                    "objective": objective,
                    "duration_seconds": investigation.get("duration_seconds"),
                    "findings_count": investigation.get("findings_count", 0),
                    "evidence_count": investigation.get("evidence_count", 0),
                    "resolution_status": resolution.get("status")
                    if resolution
                    else None,
                    "timestamp": datetime.now().isoformat(),
                    "session_id": self.supervisor_session_id,
                },
            )

            logger.info(f"Stored investigation: {objective}")

        except Exception as e:
            logger.warning(f"Failed to store investigation: {e}")

    def get_supervisor_statistics(self) -> Dict[str, Any]:
        """Obtener estadísticas del Supervisor."""
        stats = {
            "memory_enabled": self.memory_enabled,
            "session_id": self.supervisor_session_id,
            "systems": {},
        }

        # Estadísticas de ChromaDB
        if self.chroma:
            try:
                # Buscar memorias del Supervisor
                all_memories = self.chroma.search_memories(
                    query="supervisor", limit=1000
                )

                # Categorizar
                categories = {}
                for memory in all_memories:
                    mem_type = memory["metadata"].get("memory_type", "unknown")
                    categories[mem_type] = categories.get(mem_type, 0) + 1

                stats["systems"]["chromadb"] = {
                    "total_memories": len(all_memories),
                    "categories": categories,
                    "session_id": self.supervisor_session_id,
                }

            except Exception as e:
                stats["systems"]["chromadb"] = {"error": str(e)}

        # Estadísticas de Chat Memory
        if self.chat_memory:
            try:
                chat_stats = self.chat_memory.get_statistics()
                stats["systems"]["chat_memory"] = chat_stats
            except Exception as e:
                stats["systems"]["chat_memory"] = {"error": str(e)}

        return stats


# Instancia global para uso en Supervisor
_supervisor_memory: Optional[SupervisorMemoryIntegration] = None


def get_supervisor_memory() -> SupervisorMemoryIntegration:
    """Obtener instancia global de memoria del Supervisor."""
    global _supervisor_memory
    if _supervisor_memory is None:
        _supervisor_memory = SupervisorMemoryIntegration()
    return _supervisor_memory


if __name__ == "__main__":
    # Test de integración
    supervisor_mem = SupervisorMemoryIntegration()

    # Test almacenamiento
    finding = {
        "title": "Test Finding",
        "issue": "Memory integration test",
        "severity": "low",
        "component": "supervisor",
    }

    supervisor_mem.store_audit_finding(finding)

    # Test búsqueda
    similar = supervisor_mem.search_similar_issues("memory integration test")
    print(f"Similar issues found: {similar['found']}")

    # Estadísticas
    stats = supervisor_mem.get_supervisor_statistics()
    print(f"Supervisor memory stats: {stats}")
