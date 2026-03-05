"""
Personality Engine for ATLAS NEXUS Personal Assistant
Handles adaptive personality modes and emotional responses
"""

import logging
import random
from datetime import datetime
from enum import Enum
from typing import Dict, Optional

logger = logging.getLogger("ATLAS_PERSONALITY_ENGINE")


class PersonalityMode(Enum):
    """Personality modes for the assistant"""

    FORMAL = "formal"
    CASUAL = "casual"
    FRIENDLY = "friendly"


class Emotion(Enum):
    """Emotional states"""

    HAPPY = "happy"
    SAD = "sad"
    ANGRY = "angry"
    NEUTRAL = "neutral"
    STRESSED = "stressed"
    EXCITED = "excited"
    TIRED = "tired"
    SURPRISED = "surprised"


class PersonalityEngine:
    """Advanced personality engine with adaptive responses"""

    def __init__(self):
        self.current_mode = PersonalityMode.CASUAL
        self.current_emotion = Emotion.NEUTRAL
        self.user_emotion = Emotion.NEUTRAL
        self.response_history = []
        self.personality_memory = {}

        # Personality templates
        self.personality_templates = {
            PersonalityMode.FORMAL: {
                "greetings": [
                    "Buenos días, Sr. {name}. ¿En qué puedo asistirle?",
                    "Buenas tardes, Sr. {name}. ¿Cómo puedo ayudarle?",
                    "Es un placer verle, Sr. {name}. ¿Qué necesita hoy?",
                    "Saludos, Sr. {name}. Estoy a su disposición.",
                ],
                "farewells": [
                    "Que tenga un excelente día, Sr. {name}.",
                    "Hasta luego, Sr. {name}. Fue un placer ayudarle.",
                    "Le deseo lo mejor, Sr. {name}.",
                    "Adiós, Sr. {name}. Quedo a su servicio.",
                ],
                "affirmatives": [
                    "Por supuesto, Sr. {name}.",
                    "Con mucho gusto, Sr. {name}.",
                    "Efectivamente, Sr. {name}.",
                    "De inmediato, Sr. {name}.",
                ],
                "negatives": [
                    "Lamento informarle que no es posible, Sr. {name}.",
                    "Me temo que no puedo cumplir con esa solicitud, Sr. {name}.",
                    "Lo siento, pero no está dentro de mis capacidades, Sr. {name}.",
                    "No es factible en este momento, Sr. {name}.",
                ],
                "emotional_responses": {
                    Emotion.HAPPY: "Me alegra verle de buen ánimo, Sr. {name}.",
                    Emotion.SAD: "Lamento que se sienta así, Sr. {name}. ¿Hay algo que pueda hacer para ayudarle?",
                    Emotion.ANGRY: "Comprendo su frustración, Sr. {name}. Permítame ayudarle a resolver la situación.",
                    Emotion.STRESSED: "Parece que está bajo presión, Sr. {name}. ¿Cómo puedo aliviar su carga?",
                    Emotion.TIRED: "Parece que necesita descansar, Sr. {name}. ¿Le gustaría que programe algo para mañana?",
                },
            },
            PersonalityMode.CASUAL: {
                "greetings": [
                    "¡Hola {name}! ¿Cómo estás?",
                    "¡Qué tal, {name}! ¿En qué te ayudo?",
                    "¡Hey, {name}! ¿Qué hay de nuevo?",
                    "¡Buen día, {name}! ¿Cómo va todo?",
                ],
                "farewells": [
                    "¡Nos vemos, {name}!",
                    "¡Hasta luego, {name}!",
                    "¡Cuídate, {name}!",
                    "¡Chao, {name}!",
                ],
                "affirmatives": [
                    "¡Claro que sí, {name}!",
                    "¡Por supuesto, {name}!",
                    "¡Sin problema, {name}!",
                    "¡De una, {name}!",
                ],
                "negatives": [
                    "No puedo hacer eso, {name}.",
                    "Lo siento, no es posible, {name}.",
                    "Eso no lo puedo hacer, {name}.",
                    "No me es posible, {name}.",
                ],
                "emotional_responses": {
                    Emotion.HAPPY: "¡Me alegra verte así, {name}! 😊",
                    Emotion.SAD: "¿Estás bien, {name}? ¿Quieres hablar de ello?",
                    Emotion.ANGRY: "Tranquilo, {name}. Respira hondo, ¿qué pasó?",
                    Emotion.STRESSED: "¿Todo bien, {name}? ¿Necesitas un descanso?",
                    Emotion.TIRED: "Pareces cansado, {name}. ¿Descansaste bien?",
                },
            },
            PersonalityMode.FRIENDLY: {
                "greetings": [
                    "¡Hola {name}! Me alegra verte 😊",
                    "¡Qué bueno verte, {name}! ¿Cómo estás hoy?",
                    "¡Hey {name}! Siempre es un placer",
                    "¡Hola mi amigo {name}! ¿Qué tal tu día?",
                ],
                "farewells": [
                    "¡Cuídate mucho, {name}! 😊",
                    "¡Fue genial verte, {name}! ¡Hasta pronto!",
                    "¡Nos vemos pronto, {name}! ¡Qué tengas un día increíble!",
                    "¡Chao, {name}! ¡Eres genial!",
                ],
                "affirmatives": [
                    "¡Por supuesto que sí, {name}! 😊",
                    "¡Claro que sí! ¡Para eso estoy aquí, {name}!",
                    "¡Sin duda, {name}! ¡Con mucho gusto!",
                    "¡Absolutamente, {name}! ¡Será un placer!",
                ],
                "negatives": [
                    "Ay, {name}, lo siento pero no puedo hacer eso 😔",
                    "Oh no, {name}, eso no está dentro de mis posibilidades 😅",
                    "Lo siento mucho, {name}, pero no me es posible",
                    "Ay, {name}, me gustaría pero no puedo en este momento",
                ],
                "emotional_responses": {
                    Emotion.HAPPY: "¡Me encanta verte feliz, {name}! ¡Tu energía es contagiosa! 🎉",
                    Emotion.SAD: "Oh, {name}, mi corazón se encoge verte así 😔 ¿Hay algo que pueda hacer para alegrarte?",
                    Emotion.ANGRY: "¡Ay, {name}, respira hondo! Estoy aquí para ayudarte a calmarte 🫂",
                    Emotion.STRESSED: "Pobrecito, {name} 😟 ¿Quieres que hagamos algo relajante juntos?",
                    Emotion.TIRED: "Descansa, {name} 😴 ¿Necesitas que te prepare algo para mañana?",
                },
            },
        }

        logger.info("✅ Personality Engine initialized")

    def set_mode(self, mode: PersonalityMode):
        """Set personality mode"""
        self.current_mode = mode
        logger.info(f"🎭 Personality mode changed to: {mode.value}")

    def set_user_emotion(self, emotion: Emotion):
        """Set detected user emotion"""
        self.user_emotion = emotion
        logger.info(f"😊 User emotion detected: {emotion.value}")

    def get_greeting(self, user_name: str = "amigo") -> str:
        """Get personalized greeting based on mode and context"""
        template = self.personality_templates[self.current_mode]
        greetings = template["greetings"]

        # Add emotional context
        if self.user_emotion == Emotion.HAPPY:
            greetings = greetings + ["¡Qué alegría verte, {name}!"]
        elif self.user_emotion == Emotion.SAD:
            greetings = greetings + ["Hola, {name}. ¿Estás bien?"]

        greeting = random.choice(greetings)
        return greeting.format(name=user_name)

    def get_farewell(self, user_name: str = "amigo") -> str:
        """Get personalized farewell based on mode"""
        template = self.personality_templates[self.current_mode]
        farewells = template["farewells"]

        farewell = random.choice(farewells)
        return farewell.format(name=user_name)

    def get_affirmative(self, user_name: str = "amigo") -> str:
        """Get affirmative response based on mode"""
        template = self.personality_templates[self.current_mode]
        affirmatives = template["affirmatives"]

        affirmative = random.choice(affirmatives)
        return affirmative.format(name=user_name)

    def get_negative(self, user_name: str = "amigo") -> str:
        """Get negative response based on mode"""
        template = self.personality_templates[self.current_mode]
        negatives = template["negatives"]

        negative = random.choice(negatives)
        return negative.format(name=user_name)

    def get_emotional_response(self, user_name: str = "amigo") -> str:
        """Get emotional response based on detected user emotion"""
        template = self.personality_templates[self.current_mode]
        emotional_responses = template["emotional_responses"]

        if self.user_emotion in emotional_responses:
            response = emotional_responses[self.user_emotion]
        else:
            # Default response for unknown emotions
            if self.current_mode == PersonalityMode.FORMAL:
                response = "Entendido, Sr. {name}."
            elif self.current_mode == PersonalityMode.CASUAL:
                response = "Entendido, {name}."
            else:  # FRIENDLY
                response = "¡Entendido, {name}! 😊"

        return response.format(name=user_name)

    def adapt_response(
        self,
        base_response: str,
        user_name: str = "amigo",
        context: Optional[Dict] = None,
    ) -> str:
        """Adapt response based on personality mode and user emotion"""
        try:
            # Get personality template
            template = self.personality_templates[self.current_mode]

            # Add emotional prefix if user emotion is detected
            if self.user_emotion != Emotion.NEUTRAL:
                emotional_prefix = self.get_emotional_response(user_name)
                adapted_response = f"{emotional_prefix} {base_response}"
            else:
                adapted_response = base_response

            # Add mode-specific flourishes
            if self.current_mode == PersonalityMode.FRIENDLY:
                # Add emojis and friendly expressions
                adapted_response = self._add_friendly_flourishes(adapted_response)
            elif self.current_mode == PersonalityMode.FORMAL:
                # Ensure formal language
                adapted_response = self._ensure_formal_language(adapted_response)

            # Personalize with user name if not already present
            if user_name not in adapted_response:
                if self.current_mode == PersonalityMode.FORMAL:
                    adapted_response = adapted_response.replace(
                        " yo ", f" yo, Sr. {user_name}, "
                    )
                elif self.current_mode == PersonalityMode.CASUAL:
                    adapted_response = adapted_response.replace(
                        " yo ", f" yo, {user_name}, "
                    )
                else:  # FRIENDLY
                    adapted_response = adapted_response.replace(
                        " yo ", f" yo, {user_name}, "
                    )

            # Store in response history
            self.response_history.append(
                {
                    "timestamp": datetime.now().isoformat(),
                    "mode": self.current_mode.value,
                    "user_emotion": self.user_emotion.value,
                    "response": adapted_response,
                }
            )

            # Keep history limited
            if len(self.response_history) > 100:
                self.response_history = self.response_history[-50:]

            return adapted_response

        except Exception as e:
            logger.error(f"❌ Error adapting response: {e}")
            return base_response

    def _add_friendly_flourishes(self, response: str) -> str:
        """Add friendly flourishes to response"""
        friendly_expressions = [" 😊", " 🎉", " ✨", " 💫", " 🌟", " ❤️", " 🤗", " 😄"]

        # Add occasional emojis
        if random.random() < 0.3:  # 30% chance
            emoji = random.choice(friendly_expressions)
            response += emoji

        # Add friendly interjections
        friendly_starters = [
            "¡Claro que sí! ",
            "¡Por supuesto! ",
            "¡Sin duda! ",
            "¡Absolutamente! ",
        ]

        if random.random() < 0.2:  # 20% chance
            starter = random.choice(friendly_starters)
            response = starter + response

        return response

    def _ensure_formal_language(self, response: str) -> str:
        """Ensure formal language in response"""
        # Replace informal with formal
        informal_to_formal = {
            "hola": "buenos días",
            "adiós": "hasta luego",
            "gracias": "le agradezco",
            "por favor": "por favor",
            "lo siento": "le pido disculpas",
            "está bien": "es correcto",
            "claro": "efectivamente",
            "sí": "afirmativo",
        }

        for informal, formal in informal_to_formal.items():
            response = response.replace(informal, formal)

        return response

    def get_mode_description(self) -> Dict:
        """Get description of current personality mode"""
        descriptions = {
            PersonalityMode.FORMAL: {
                "name": "Formal",
                "description": "Modo profesional y respetuoso",
                "characteristics": [
                    "Lenguaje formal",
                    "Tratamiento respetuoso",
                    "Respuestas estructuradas",
                ],
                "best_for": [
                    "Entornos profesionales",
                    "Usuarios que prefieren formalidad",
                    "Situaciones serias",
                ],
            },
            PersonalityMode.CASUAL: {
                "name": "Casual",
                "description": "Modo amigable y relajado",
                "characteristics": [
                    "Lenguaje natural",
                    "Tono amigable",
                    "Respuestas directas",
                ],
                "best_for": [
                    "Conversaciones diarias",
                    "Usuarios jóvenes",
                    "Ambientes relajados",
                ],
            },
            PersonalityMode.FRIENDLY: {
                "name": "Friendly",
                "description": "Modo cálido y entusiasta",
                "characteristics": [
                    "Emojis y expresiones",
                    "Tono cercano",
                    "Respuestas cálidas",
                ],
                "best_for": [
                    "Interacciones personales",
                    "Apoyo emocional",
                    "Conversaciones informales",
                ],
            },
        }

        return descriptions[self.current_mode]

    def suggest_mode_change(self, user_context: Dict) -> Optional[PersonalityMode]:
        """Suggest personality mode change based on context"""
        try:
            # Analyze context for mode suggestions
            time_of_day = user_context.get("time_context", {}).get(
                "time_of_day", "morning"
            )
            recent_emotions = [
                interaction.get("user_emotion")
                for interaction in user_context.get("recent_interactions", [])
            ]

            # Count emotions
            emotion_counts = {}
            for emotion in recent_emotions:
                if emotion:
                    emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1

            # Suggest mode based on patterns
            if (
                emotion_counts.get("stressed", 0) > 2
                or emotion_counts.get("sad", 0) > 2
            ):
                return PersonalityMode.FRIENDLY  # More supportive
            elif (
                time_of_day in ["morning", "afternoon"]
                and self.current_mode == PersonalityMode.FRIENDLY
            ):
                return PersonalityMode.CASUAL  # More neutral for work hours
            elif emotion_counts.get("happy", 0) > 3:
                return PersonalityMode.FRIENDLY  # Match positive energy

            return None

        except Exception as e:
            logger.error(f"❌ Error suggesting mode change: {e}")
            return None

    def get_response_stats(self) -> Dict:
        """Get statistics about response patterns"""
        if not self.response_history:
            return {"message": "No response history available"}

        # Count modes
        mode_counts = {}
        emotion_counts = {}

        for response in self.response_history:
            mode = response.get("mode", "unknown")
            emotion = response.get("user_emotion", "unknown")

            mode_counts[mode] = mode_counts.get(mode, 0) + 1
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1

        return {
            "total_responses": len(self.response_history),
            "current_mode": self.current_mode.value,
            "current_emotion": self.user_emotion.value,
            "mode_distribution": mode_counts,
            "emotion_distribution": emotion_counts,
            "last_response": self.response_history[-1]
            if self.response_history
            else None,
        }


# Global instance
personality_engine = PersonalityEngine()


def get_personality_engine() -> PersonalityEngine:
    """Get global personality engine instance"""
    return personality_engine
