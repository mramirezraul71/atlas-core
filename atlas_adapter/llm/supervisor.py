"""Supervisor - Operational mode for ATLAS Owner (Raúl)."""
import os
from typing import Dict, Any, List
from .router import LLMRouter
from .audit import AuditLogger


class Supervisor:
    """
    Supervisor operacional subordinado al Owner (Raúl).
    Estilo anti-corporativo, directo, técnico.
    """
    
    def __init__(self):
        self.router = LLMRouter()
        self.audit = AuditLogger()
        self.owner = "Raúl"
        self.style = os.getenv("ATLAS_SUPERVISOR_STYLE", "operational")
    
    def advise(self, objective: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate operational advice for the Owner.
        
        Args:
            objective: The objective or question to address
            context: Context dictionary with system state, issues, modules, etc.
            
        Returns:
            Dict with recommendations, prompts, and analysis
        """
        # Audit log with style and objective
        self.audit.log_event("supervisor_request", {
            "style": self.style,
            "objective": objective,
            "context_keys": list(context.keys())
        })
        
        # Build operational prompt
        prompt = self._build_prompt(objective, context)
        
        # Get LLM response with task "auditoria_tecnica"
        llm_result = self.router.generate(
            prompt=prompt,
            context={"task": "auditoria_tecnica", **context}
        )
        
        if not llm_result["ok"]:
            return {
                "ok": False,
                "error": llm_result.get("error"),
                "recommendations": [],
                "prompts": [],
                "analysis": "Error en LLM - no se pudo generar análisis"
            }
        
        # Parse and structure the response
        response_text = llm_result.get("response", "")
        
        return {
            "ok": True,
            "recommendations": self._extract_recommendations(response_text),
            "prompts": self._extract_prompts(response_text),
            "analysis": response_text,
            "provider": llm_result.get("provider"),
            "model": llm_result.get("model"),
            "fallback_used": llm_result.get("fallback_used", False)
        }
    
    def _build_prompt(self, objective: str, context: Dict[str, Any]) -> str:
        """Build operational prompt with anti-corporate style."""
        issues = context.get("issues", [])
        modules = context.get("modules", [])
        metrics = context.get("metrics", {})
        
        prompt = f"""SYSTEM: Eres el Supervisor técnico de ATLAS, subordinado al Owner {self.owner}.

ESTILO OPERACIONAL:
- Sin formalidades corporativas
- Directo y técnico
- Diagnóstico preciso
- Acción inmediata
- Cero ambigüedad

OBJETIVO: {objective}

CONTEXTO:
- Issues: {', '.join(issues) if issues else 'Ninguno'}
- Módulos: {', '.join(modules) if modules else 'Ninguno'}
- Métricas: {metrics if metrics else 'N/A'}

FORMATO OBLIGATORIO:

## DIAGNÓSTICO
[Qué está pasando - máximo 3 líneas]

## ACCIÓN INMEDIATA
- [Acción 1 - específica y ejecutable]
- [Acción 2 - específica y ejecutable]
- [Acción 3 - específica y ejecutable]

## CORRECCIÓN
[Qué arreglar para evitar recurrencia]

## VALIDACIÓN
[Cómo verificar que se resolvió]

## PROMPT
- [Comando/pregunta técnica 1]
- [Comando/pregunta técnica 2]

Responde SIN introducciones, SIN conclusiones, SIN relleno."""
        
        return prompt
    
    def _extract_recommendations(self, text: str) -> List[str]:
        """Extract recommendations from LLM response."""
        recommendations = []
        
        # Simple extraction - look for lines starting with "- " after "Recomendaciones"
        in_recommendations = False
        for line in text.split("\n"):
            line = line.strip()
            
            if "recomendacion" in line.lower() and ("##" in line or "**" in line):
                in_recommendations = True
                continue
            
            if in_recommendations:
                if line.startswith("- "):
                    recommendations.append(line[2:].strip())
                elif line.startswith("##") or line.startswith("**"):
                    in_recommendations = False
        
        # If no structured recommendations found, return generic ones
        if not recommendations:
            recommendations = [
                "Revisar logs del sistema para identificar errores",
                "Verificar conectividad de módulos críticos",
                "Ejecutar diagnóstico completo del sistema"
            ]
        
        return recommendations
    
    def _extract_prompts(self, text: str) -> List[str]:
        """Extract suggested prompts from LLM response."""
        prompts = []
        
        # Simple extraction - look for lines starting with "- " after "Prompts"
        in_prompts = False
        for line in text.split("\n"):
            line = line.strip()
            
            if "prompt" in line.lower() and ("##" in line or "**" in line):
                in_prompts = True
                continue
            
            if in_prompts:
                if line.startswith("- "):
                    prompts.append(line[2:].strip())
                elif line.startswith("##") or line.startswith("**"):
                    in_prompts = False
        
        # If no structured prompts found, return generic ones
        if not prompts:
            prompts = [
                "Analizar estado de salud de todos los módulos",
                "Revisar incidentes recientes y patrones",
                "Proponer optimizaciones de rendimiento"
            ]
        
        return prompts
