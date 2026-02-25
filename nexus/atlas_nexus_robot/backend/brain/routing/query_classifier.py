"""
ATLAS NEXUS - Query Classifier
Clasificación inteligente de queries para routing automático
"""

import logging
import re
from enum import Enum
from typing import Dict, List, Tuple

logger = logging.getLogger(__name__)


class QueryType(Enum):
    """Tipos de queries soportados"""

    CODE = "code"
    REASONING = "reasoning"
    ANALYSIS = "analysis"
    GENERAL = "general"


class QueryClassifier:
    """Clasificador de queries usando patrones y análisis de contenido"""

    def __init__(self):
        """Inicializa el clasificador con patrones predefinidos"""
        self.patterns = {
            QueryType.CODE: [
                r"\b(código|función|script|debug|programar|python|javascript|java|c\+\+|html|css|react|node|api|algoritmo|clase|método|variable|bucle|if|else|for|while|def|return|import|export|async|await|promise|array|object|string|number|boolean)\b",
                r"\b(escribe|crea|desarrolla|implementa|codifica|programa)\s+\b(un|una|el|la)\s+\b(función|script|código|programa|algoritmo|método)\b",
                r"\b(error|bug|exception|fail|issue|problem)\s+\b(en|in|del|de la)\s+\b(código|programa|script|función)\b",
                r"\b(how\s+to|cómo\s+|how\s+do\s+i)\s+\b(codificar|programar|crear|implementar|escribir)\b",
                r"\b(sintaxis|syntax|semántica|semantics)\s+\b(del|de|in)\s+\b\w+\b",
                r"\b(library|librería|framework|paquete|package|module|módulo)\b",
                r"\b(test|prueba|unit|unittest)\s+\b(código|programa|función)\b",
            ],
            QueryType.REASONING: [
                r"\b(calcula|matemática|ecuación|resuelve|problema|lógica|razonamiento|cuánto\s+es|suma|resta|multiplica|divide|porcentaje|derivada|integral|promedio|media|mediana|moda|varianza|desviación)\b",
                r"\b\d+\s*[\+\-\*\/]\s*\d+\b",  # Operaciones matemáticas básicas
                r"\b(\d+\.?\d*)\s*(por|de)\s*(\d+\.?\d*)\s*%?\b",  # Porcentajes
                r"\b(es\s+igual\s+a|=\s*\?|resulta|da\s+como)\b",
                r"\b(pensar|razonar|analizar|evaluar|calcular|determinar|encontrar)\s+\b(la\s+)?(solución|respuesta|resultado)\b",
                r"\b(step|pasos|procedimiento|método)\s+\b(para|to)\s+\b(resolver|calcular)\b",
                r"\b(formula|fórmula|teorema|ley|principio)\b",
            ],
            QueryType.ANALYSIS: [
                r"\b(resume|resumen|analiza\s+texto|traduce|interpreta|explica|compara|evalúa|sintetiza|parafrasea|extrae|identifica|describe|detalla)\b",
                r"\b(qué\s+es|cuál\s+es|define|significa|representa)\b",
                r"\b(diferencia|similaridad|comparación|ventajas|desventajas)\s+\b(entre|between)\b",
                r"\b(análisis|interpretación|evaluación|crítica|reseña)\s+\b(de|del)\b",
                r"\b(puntos\s+clave|aspectos\s+importantes|características|elementos)\b",
                r"\b(significado|contexto|implicación|relevancia|importancia)\b",
                r"\b(estructura|organización|formato|esquema)\s+\b(del|de)\s+\b(texto|documento|información)\b",
            ],
            QueryType.GENERAL: [
                r"\b(hola|adiós|gracias|cómo\s+estás|qué\s+tal|ayuda|información|dime|explícame|cuéntame|buenos\s+días|buenas\s+tardes|buenas\s+noches)\b",
                r"\b(quién|qué|cuándo|dónde|por\s+qué|cómo)\s+\b(es|son|fue|fueron|será|serán)\b",
                r"\b(puedes|podrías|puedo|podría)\s+\b(ayudarme|decirme|explicarme|mostrarme)\b",
                r"\b(me\s+gustaría|quiero|necesito)\s+\b(saber|conocer|entender)\b",
                r"\b(opinión|recomendación|sugerencia|consejo)\b",
                r"\b(historia|cuento|chiste|anécdota)\b",
            ],
        }

        # Pesos para cada tipo (mayor peso = más prioridad)
        self.type_weights = {
            QueryType.CODE: 3.0,
            QueryType.REASONING: 2.5,
            QueryType.ANALYSIS: 2.0,
            QueryType.GENERAL: 1.0,
        }

    def classify_query(self, query: str) -> Tuple[QueryType, float]:
        """
        Clasifica una query y retorna el tipo y nivel de confianza

        Args:
            query: Texto de la query del usuario

        Returns:
            Tuple con (QueryType, confidence_score)
        """
        if not query or not query.strip():
            return QueryType.GENERAL, 0.0

        query_lower = query.lower()
        scores = {}

        # Calcular score para cada tipo
        for query_type, patterns in self.patterns.items():
            score = 0.0
            matches = 0

            for pattern in patterns:
                try:
                    pattern_matches = len(
                        re.findall(pattern, query_lower, re.IGNORECASE)
                    )
                    if pattern_matches > 0:
                        # Score basado en número de matches y peso del tipo
                        score += pattern_matches * self.type_weights[query_type]
                        matches += pattern_matches
                except re.error as e:
                    logger.warning(f"Pattern error for {query_type}: {e}")
                    continue

            # Normalizar score por longitud del query
            if len(query_lower) > 0:
                scores[query_type] = score / len(query_lower)
            else:
                scores[query_type] = 0.0

        # Encontrar el tipo con mayor score
        if not scores:
            return QueryType.GENERAL, 0.0

        best_type = max(scores.keys(), key=lambda k: scores[k])
        confidence = min(scores[best_type], 1.0)  # Limitar a 1.0

        # Si la confianza es muy baja, default a GENERAL
        if confidence < 0.1:
            return QueryType.GENERAL, 0.0

        return best_type, confidence

    def get_classification_details(self, query: str) -> Dict:
        """
        Retorna detalles completos de la clasificación

        Args:
            query: Texto de la query del usuario

        Returns:
            Diccionario con detalles de clasificación
        """
        query_type, confidence = self.classify_query(query)

        return {
            "query": query,
            "type": query_type.value,
            "confidence": confidence,
            "all_scores": self._calculate_all_scores(query),
            "matched_patterns": self._get_matched_patterns(query, query_type),
            "recommended_model": self._get_recommended_model(query_type),
        }

    def _calculate_all_scores(self, query: str) -> Dict[str, float]:
        """Calcula scores para todos los tipos"""
        query_lower = query.lower()
        scores = {}

        for query_type, patterns in self.patterns.items():
            score = 0.0
            for pattern in patterns:
                try:
                    matches = len(re.findall(pattern, query_lower, re.IGNORECASE))
                    if matches > 0:
                        score += matches * self.type_weights[query_type]
                except re.error:
                    continue

            if len(query_lower) > 0:
                scores[query_type.value] = min(score / len(query_lower), 1.0)
            else:
                scores[query_type.value] = 0.0

        return scores

    def _get_matched_patterns(self, query: str, query_type: QueryType) -> List[str]:
        """Retorna los patrones que hicieron match para el tipo seleccionado"""
        query_lower = query.lower()
        matched = []

        if query_type in self.patterns:
            for pattern in self.patterns[query_type]:
                try:
                    if re.search(pattern, query_lower, re.IGNORECASE):
                        matched.append(pattern)
                except re.error:
                    continue

        return matched

    def _get_recommended_model(self, query_type: QueryType) -> str:
        """Retorna el modelo recomendado para un tipo de query"""
        recommendations = {
            QueryType.CODE: "deepseek-coder",
            QueryType.REASONING: "deepseek-r1",
            QueryType.ANALYSIS: "mistral",
            QueryType.GENERAL: "llama3.2",
        }
        return recommendations.get(query_type, "llama3.2")


# Singleton instance
_query_classifier = None


def get_query_classifier() -> QueryClassifier:
    """Get global query classifier instance"""
    global _query_classifier
    if _query_classifier is None:
        _query_classifier = QueryClassifier()
    return _query_classifier
