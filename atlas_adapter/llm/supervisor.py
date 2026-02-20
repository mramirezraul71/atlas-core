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
        self.audit.log("supervisor_request", {
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
        """Build operational prompt with ATLAS_RUN JSON format."""
        issues = context.get("issues", [])
        modules = context.get("modules", [])
        metrics = context.get("metrics", {})
        
        prompt = f"""SYSTEM: Eres el Supervisor técnico de ATLAS, subordinado al Owner {self.owner}.

ESTILO OPERACIONAL:
- Sin formalidades corporativas
- Directo y técnico
- Diagnóstico preciso
- Plan de ejecución automático
- Cero ambigüedad

OBJETIVO: {objective}

CONTEXTO:
- Issues: {', '.join(issues) if issues else 'Ninguno'}
- Módulos: {', '.join(modules) if modules else 'Ninguno'}
- Métricas: {metrics if metrics else 'N/A'}

PROHIBIDO:
- Sección "PROMPTS SUGERIDOS"
- Listas de prompts en texto

OBLIGATORIO:
- Termina SIEMPRE tu respuesta con:
ATLAS_RUN:
{{JSON válido}}

JSON SPEC:
- Debe ser JSON estricto.
- Debe incluir: version, mode, steps[]
- Cada step: id, risk ("low"|"high"), type ("terminal"), cmd
- Para Workspace usa type:"terminal" y cmd con comandos reales de ATLAS.

FORMATO OBLIGATORIO (CRÍTICO - DEBES SEGUIRLO EXACTAMENTE):

## DIAGNÓSTICO
[Máximo 5 líneas - qué está pasando y por qué]

ATLAS_RUN:
{{
  "version": "1",
  "mode": "execute",
  "steps": [
    {{"id": "s1", "risk": "low", "type": "terminal", "cmd": "atlas status"}},
    {{"id": "s2", "risk": "low", "type": "terminal", "cmd": "atlas modules list --status"}},
    {{"id": "s3", "risk": "high", "type": "terminal", "cmd": "atlas restart telegram"}}
  ]
}}

REGLAS PARA ATLAS_RUN:
1. SIEMPRE incluir el bloque ATLAS_RUN con JSON válido
2. Usar comandos ATLAS reales: atlas status, atlas modules list, atlas restart <module>, etc.
3. risk="low" para comandos de lectura/diagnóstico (status, list, logs)
4. risk="high" para comandos que modifican estado (restart, stop, config)
5. type="terminal" para comandos shell
6. type="http" para llamadas API (ej: GET http://127.0.0.1:8791/health)
7. NO incluir sección "PROMPT" ni "PROMPTS SUGERIDOS"
8. Mínimo 3 steps, máximo 8 steps

Responde SOLO con DIAGNÓSTICO + ATLAS_RUN. SIN introducciones, SIN conclusiones."""
        
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
