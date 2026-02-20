"""Supervisor - Generates recommendations and prompts for ATLAS Owner (Raúl)."""
from typing import Dict, Any, List
from .router import LLMRouter


class Supervisor:
    """
    Supervisor subordinado al Owner (Raúl).
    Genera recomendaciones y prompts basados en el estado del sistema.
    """
    
    def __init__(self):
        self.router = LLMRouter()
        self.owner = "Raúl"
    
    def advise(self, objective: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate advice and recommendations for the Owner.
        
        Args:
            objective: The objective or question to address
            context: Context dictionary with system state, issues, modules, etc.
            
        Returns:
            Dict with recommendations, prompts, and analysis
        """
        # Build comprehensive prompt for the LLM
        prompt = self._build_prompt(objective, context)
        
        # Get LLM response
        llm_result = self.router.generate(prompt, context)
        
        if not llm_result["ok"]:
            return {
                "ok": False,
                "error": llm_result.get("error"),
                "recommendations": [],
                "prompts": [],
                "analysis": "No se pudo generar análisis debido a error en LLM"
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
        """Build a comprehensive prompt for the LLM."""
        issues = context.get("issues", [])
        modules = context.get("modules", [])
        metrics = context.get("metrics", {})
        
        prompt = f"""Eres el Supervisor de ATLAS, subordinado al Owner {self.owner}.

OBJETIVO: {objective}

CONTEXTO DEL SISTEMA:
- Issues reportados: {', '.join(issues) if issues else 'Ninguno'}
- Módulos involucrados: {', '.join(modules) if modules else 'Ninguno'}
- Métricas: {metrics if metrics else 'No disponibles'}

INSTRUCCIONES:
1. Analiza la situación actual
2. Identifica problemas y oportunidades
3. Genera recomendaciones concretas y accionables
4. Propón prompts específicos para investigar o resolver issues

FORMATO DE RESPUESTA:
## Análisis
[Tu análisis aquí]

## Recomendaciones
- [Recomendación 1]
- [Recomendación 2]
- [Recomendación 3]

## Prompts Sugeridos
- [Prompt 1 para investigar X]
- [Prompt 2 para resolver Y]

Responde de forma concisa y profesional."""
        
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
