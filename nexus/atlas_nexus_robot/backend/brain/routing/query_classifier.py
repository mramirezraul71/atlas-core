"""
ATLAS NEXUS - Query Classifier
Clasificacion inteligente de queries para routing automatico.
"""

import logging
import re
import unicodedata
from enum import Enum
from typing import Any, Dict, List, Tuple

logger = logging.getLogger(__name__)


class QueryType(Enum):
    """Tipos de queries soportados."""

    CODE = "code"
    REASONING = "reasoning"
    ANALYSIS = "analysis"
    GENERAL = "general"


class QueryClassifier:
    """Clasificador de queries usando patrones + heuristicas de complejidad."""

    def __init__(self):
        self.patterns = {
            QueryType.CODE: [
                r"\b(codigo|funcion|script|debug|programar|python|javascript|java|c\+\+|html|css|react|node|api|algoritmo|clase|metodo|variable|bucle|if|else|for|while|def|return|import|export|async|await|promise|array|object|string|number|boolean|arquitectura|microservicios|pipeline|refactor|escalable|monorepo)\b",
                r"\b(escribe|crea|desarrolla|implementa|codifica|programa)\s+\b(un|una|el|la)\s+\b(funcion|script|codigo|programa|algoritmo|metodo)\b",
                r"\b(error|bug|exception|fail|issue|problem)\s+\b(en|in|del|de la)\s+\b(codigo|programa|script|funcion)\b",
                r"\b(how\s+to|como\s+|how\s+do\s+i)\s+\b(codificar|programar|crear|implementar|escribir)\b",
                r"\b(sintaxis|syntax|semantica|semantics)\s+\b(del|de|in)\s+\b\w+\b",
                r"\b(library|libreria|framework|paquete|package|module|modulo)\b",
                r"\b(test|prueba|unit|unittest)\s+\b(codigo|programa|funcion)\b",
            ],
            QueryType.REASONING: [
                r"\b(calcula|matematica|ecuacion|resuelve|problema|logica|razonamiento|cuanto\s+es|suma|resta|multiplica|divide|porcentaje|derivada|integral|promedio|media|mediana|moda|varianza|desviacion)\b",
                r"\b\d+\s*[\+\-\*\/]\s*\d+\b",
                r"\b(\d+\.?\d*)\s*(por|de)\s*(\d+\.?\d*)\s*%?\b",
                r"\b(es\s+igual\s+a|=\s*\?|resulta|da\s+como)\b",
                r"\b(pensar|razonar|analizar|evaluar|calcular|determinar|encontrar)\s+\b(la\s+)?(solucion|respuesta|resultado)\b",
                r"\b(step|paso|pasos|procedimiento|metodo)\s+\b(para|to)\s+\b(resolver|calcular)\b",
                r"\b(formula|teorema|ley|principio)\b",
            ],
            QueryType.ANALYSIS: [
                r"\b(resume|resumen|analiza\s+texto|traduce|interpreta|explica|compara|evalua|sintetiza|parafrasea|extrae|identifica|describe|detalla)\b",
                r"\b(que\s+es|cual\s+es|define|significa|representa)\b",
                r"\b(diferencia|similaridad|comparacion|ventajas|desventajas)\s+\b(entre|between)\b",
                r"\b(analisis|interpretacion|evaluacion|critica|resena)\s+\b(de|del)\b",
                r"\b(puntos\s+clave|aspectos\s+importantes|caracteristicas|elementos)\b",
                r"\b(significado|contexto|implicacion|relevancia|importancia)\b",
                r"\b(estructura|organizacion|formato|esquema)\s+\b(del|de)\s+\b(texto|documento|informacion)\b",
            ],
            QueryType.GENERAL: [
                r"\b(hola|adios|gracias|como\s+estas|que\s+tal|ayuda|informacion|dime|explicame|cuentame|buenos\s+dias|buenas\s+tardes|buenas\s+noches)\b",
                r"\b(quien|que|cuando|donde|por\s+que|como)\s+\b(es|son|fue|fueron|sera|seran)\b",
                r"\b(puedes|podrias|puedo|podria)\s+\b(ayudarme|decirme|explicarme|mostrarme)\b",
                r"\b(me\s+gustaria|quiero|necesito)\s+\b(saber|conocer|entender)\b",
                r"\b(opinion|recomendacion|sugerencia|consejo)\b",
                r"\b(historia|cuento|chiste|anecdota)\b",
            ],
        }

        self.type_weights = {
            QueryType.CODE: 3.0,
            QueryType.REASONING: 2.5,
            QueryType.ANALYSIS: 2.0,
            QueryType.GENERAL: 1.0,
        }

    def classify_query(self, query: str) -> Tuple[QueryType, float]:
        """Clasifica query y retorna (tipo, confianza)."""
        if not query or not query.strip():
            return QueryType.GENERAL, 0.0

        query_lower = self._normalize_query(query)
        scores = {}

        norm_words = [w for w in re.split(r"\s+", query_lower.strip()) if w]
        norm_base = max(1, len(norm_words))

        for query_type, patterns in self.patterns.items():
            score = 0.0
            for pattern in patterns:
                try:
                    pattern_matches = len(
                        re.findall(pattern, query_lower, re.IGNORECASE)
                    )
                    if pattern_matches > 0:
                        score += pattern_matches * self.type_weights[query_type]
                except re.error as err:
                    logger.warning("Pattern error for %s: %s", query_type, err)
                    continue

            scores[query_type] = score / norm_base if query_lower else 0.0

        if not scores:
            return QueryType.GENERAL, 0.0

        best_type = max(scores.keys(), key=lambda key: scores[key])
        confidence = min(scores[best_type], 1.0)
        if confidence < 0.1:
            return QueryType.GENERAL, 0.0
        return best_type, confidence

    def analyze_query(self, query: str) -> Dict[str, Any]:
        """
        Analiza query con clasificacion + complejidad para smart routing.
        """
        query_type, confidence = self.classify_query(query)
        complexity_level, complexity_score, complexity_signals = (
            self._estimate_complexity(query, query_type, confidence)
        )
        return {
            "type": query_type.value,
            "confidence": confidence,
            "complexity_level": complexity_level,
            "complexity_score": complexity_score,
            "complexity_signals": complexity_signals,
        }

    def get_classification_details(self, query: str) -> Dict[str, Any]:
        """Retorna detalles completos de clasificacion y complejidad."""
        analysis = self.analyze_query(query)
        query_type = QueryType(analysis["type"])
        return {
            "query": query,
            "type": analysis["type"],
            "confidence": analysis["confidence"],
            "complexity_level": analysis["complexity_level"],
            "complexity_score": analysis["complexity_score"],
            "complexity_signals": analysis["complexity_signals"],
            "all_scores": self._calculate_all_scores(query),
            "matched_patterns": self._get_matched_patterns(query, query_type),
            "recommended_model": self._get_recommended_model(query_type),
        }

    def _calculate_all_scores(self, query: str) -> Dict[str, float]:
        query_lower = self._normalize_query(query)
        scores = {}
        norm_words = [w for w in re.split(r"\s+", query_lower.strip()) if w]
        norm_base = max(1, len(norm_words))

        for query_type, patterns in self.patterns.items():
            score = 0.0
            for pattern in patterns:
                try:
                    matches = len(re.findall(pattern, query_lower, re.IGNORECASE))
                    if matches > 0:
                        score += matches * self.type_weights[query_type]
                except re.error:
                    continue
            scores[query_type.value] = min(score / norm_base, 1.0) if query_lower else 0.0
        return scores

    def _get_matched_patterns(self, query: str, query_type: QueryType) -> List[str]:
        query_lower = self._normalize_query(query)
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
        recommendations = {
            QueryType.CODE: "deepseek-coder-v2:16b",
            QueryType.REASONING: "qwen3-coder:30b",
            QueryType.ANALYSIS: "llama3.1:8b",
            QueryType.GENERAL: "llama3.1:8b",
        }
        return recommendations.get(query_type, "llama3.1:8b")

    def _estimate_complexity(
        self, query: str, query_type: QueryType, confidence: float
    ) -> Tuple[str, float, Dict[str, Any]]:
        """Heuristica rapida de complejidad para fast-path vs heavy-path."""
        if not query or not query.strip():
            return "simple", 0.0, {"empty_query": True}

        query_lower = self._normalize_query(query)
        words = [w for w in re.split(r"\s+", query_lower.strip()) if w]
        word_count = len(words)
        score = 0.0
        signals: Dict[str, Any] = {
            "word_count": word_count,
            "query_type": query_type.value,
        }

        if word_count >= 25:
            score += 0.14
            signals["long_query"] = True
        if word_count >= 60:
            score += 0.22
            signals["very_long_query"] = True
        if "\n" in query:
            score += 0.09
            signals["multiline"] = True
        if "```" in query:
            score += 0.25
            signals["code_block"] = True

        punctuation_count = sum(query.count(ch) for ch in ";:(){}[]")
        if punctuation_count >= 8:
            score += 0.08
            signals["structured_prompt"] = True

        if query_type in (QueryType.CODE, QueryType.REASONING):
            score += 0.10

        heavy_patterns = {
            QueryType.CODE: [
                r"\b(arquitectura|refactor|optimiza|escalable|microservicio|pipeline)\b",
                r"\b(multiarchivo|monorepo|integration|integracion|deploy)\b",
                r"\b(pruebas|tests?|debug complejo|concurrencia|performance)\b",
            ],
            QueryType.REASONING: [
                r"\b(paso a paso|demuestra|justifica|trade[- ]?off|hipotesis)\b",
                r"\b(probabilidad|estrategia|simulacion|analisis cuantitativo)\b",
            ],
            QueryType.ANALYSIS: [
                r"\b(compara.*(profundo|detallado)|sintetiza|evalua riesgos)\b",
                r"\b(reporte ejecutivo|matriz|diagnostico completo)\b",
            ],
            QueryType.GENERAL: [
                r"\b(plan completo|roadmap|estrategia completa|disena sistema)\b",
            ],
        }

        matched_heavy = 0
        for pattern in heavy_patterns.get(query_type, []):
            try:
                matched_heavy += len(re.findall(pattern, query_lower, re.IGNORECASE))
            except re.error:
                continue
        if matched_heavy:
            score += min(0.35, 0.17 + (0.06 * matched_heavy))
            signals["heavy_pattern_matches"] = matched_heavy

        op_count = len(re.findall(r"[+\-*/=<>]", query))
        if op_count >= 8:
            score += 0.10
            signals["dense_math_or_logic"] = True

        if confidence < 0.18:
            score += 0.06
            signals["low_classification_confidence"] = True

        score = max(0.0, min(score, 1.0))
        if score < 0.35:
            level = "simple"
        elif score < 0.68:
            level = "medium"
        else:
            level = "complex"
        return level, score, signals

    def _normalize_query(self, query: str) -> str:
        normalized = unicodedata.normalize("NFKD", query.lower())
        return "".join(ch for ch in normalized if not unicodedata.combining(ch))


_query_classifier = None


def get_query_classifier() -> QueryClassifier:
    """Get global query classifier instance."""
    global _query_classifier
    if _query_classifier is None:
        _query_classifier = QueryClassifier()
    return _query_classifier
