"""
Modelos de datos para el sistema de chats de ATLAS
"""

import uuid
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional


class TipoMensaje(str, Enum):
    """Tipos de mensajes"""

    TEXTO = "texto"
    COMANDO = "comando"
    NOTIFICACION = "notificacion"
    SISTEMA = "sistema"
    ARCHIVO = "archivo"
    AUDIO = "audio"


class RolParticipante(str, Enum):
    """Roles de participantes en chats"""

    ESPECIALISTA = "especialista"
    ATLAS = "atlas"
    SISTEMA = "sistema"
    USUARIO = "usuario"
    BOT = "bot"


class EstadoConversacion(str, Enum):
    """Estados de una conversación"""

    ACTIVA = "ACTIVA"
    PAUSADA = "PAUSADA"
    CERRADA = "CERRADA"
    ARCHIVADA = "ARCHIVADA"


@dataclass
class Mensaje:
    """Representa un mensaje en una conversación"""

    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    conversacion_id: str = ""
    remitente: str = ""
    rol: RolParticipante = RolParticipante.USUARIO
    contenido: str = ""
    timestamp: datetime = field(default_factory=datetime.now)
    tipo: TipoMensaje = TipoMensaje.TEXTO
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convierte el mensaje a diccionario"""
        return {
            "id": self.id,
            "conversacion_id": self.conversacion_id,
            "remitente": self.remitente,
            "rol": self.rol.value,
            "contenido": self.contenido,
            "timestamp": self.timestamp.isoformat(),
            "tipo": self.tipo.value,
            "metadata": self.metadata,
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> "Mensaje":
        """Crea un mensaje desde un diccionario"""
        msg = Mensaje(
            id=data.get("id", str(uuid.uuid4())),
            conversacion_id=data.get("conversacion_id", ""),
            remitente=data.get("remitente", ""),
            rol=RolParticipante(data.get("rol", "usuario")),
            contenido=data.get("contenido", ""),
            tipo=TipoMensaje(data.get("tipo", "texto")),
            metadata=data.get("metadata", {}),
        )
        if isinstance(data.get("timestamp"), str):
            msg.timestamp = datetime.fromisoformat(data["timestamp"])
        return msg


@dataclass
class Conversacion:
    """Representa una conversación (hilo de chat)"""

    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    titulo: str = ""
    especialista_id: Optional[str] = None
    visita_id: Optional[str] = None
    fecha_inicio: datetime = field(default_factory=datetime.now)
    fecha_fin: Optional[datetime] = None
    estado: EstadoConversacion = EstadoConversacion.ACTIVA
    tema: str = ""
    resumen: str = ""
    mensajes: List[Mensaje] = field(default_factory=list)
    participantes: List[str] = field(default_factory=list)

    def agregar_mensaje(self, mensaje: Mensaje) -> None:
        """Agrega un mensaje a la conversación"""
        mensaje.conversacion_id = self.id
        self.mensajes.append(mensaje)
        if mensaje.remitente not in self.participantes:
            self.participantes.append(mensaje.remitente)

    def obtener_historial(self, limite: int = None) -> List[Mensaje]:
        """Obtiene el historial de mensajes"""
        if limite:
            return self.mensajes[-limite:]
        return self.mensajes

    def cerrar(self) -> None:
        """Cierra la conversación"""
        self.estado = EstadoConversacion.CERRADA
        self.fecha_fin = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convierte la conversación a diccionario"""
        return {
            "id": self.id,
            "titulo": self.titulo,
            "especialista_id": self.especialista_id,
            "visita_id": self.visita_id,
            "fecha_inicio": self.fecha_inicio.isoformat(),
            "fecha_fin": self.fecha_fin.isoformat() if self.fecha_fin else None,
            "estado": self.estado.value,
            "tema": self.tema,
            "resumen": self.resumen,
            "total_mensajes": len(self.mensajes),
            "participantes": self.participantes,
            "mensajes": [m.to_dict() for m in self.mensajes],
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> "Conversacion":
        """Crea una conversación desde un diccionario"""
        conv = Conversacion(
            id=data.get("id", str(uuid.uuid4())),
            titulo=data.get("titulo", ""),
            especialista_id=data.get("especialista_id"),
            visita_id=data.get("visita_id"),
            estado=EstadoConversacion(data.get("estado", "ACTIVA")),
            tema=data.get("tema", ""),
            resumen=data.get("resumen", ""),
            participantes=data.get("participantes", []),
        )
        if isinstance(data.get("fecha_inicio"), str):
            conv.fecha_inicio = datetime.fromisoformat(data["fecha_inicio"])
        if isinstance(data.get("fecha_fin"), str):
            conv.fecha_fin = datetime.fromisoformat(data["fecha_fin"])

        # Cargar mensajes
        for msg_data in data.get("mensajes", []):
            conv.mensajes.append(Mensaje.from_dict(msg_data))

        return conv


@dataclass
class EventoAuditoria:
    """Evento de auditoría de chat"""

    id: int = field(default_factory=lambda: int(datetime.now().timestamp() * 1000))
    conversacion_id: str = ""
    mensaje_id: Optional[str] = None
    accion: str = ""  # crear, enviar, editar, eliminar, cerrar
    actor: str = ""
    timestamp: datetime = field(default_factory=datetime.now)
    detalles: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convierte el evento a diccionario"""
        return {
            "id": self.id,
            "conversacion_id": self.conversacion_id,
            "mensaje_id": self.mensaje_id,
            "accion": self.accion,
            "actor": self.actor,
            "timestamp": self.timestamp.isoformat(),
            "detalles": self.detalles,
        }


@dataclass
class ResultadoBusqueda:
    """Resultado de búsqueda en chats"""

    total: int = 0
    mensajes: List[Mensaje] = field(default_factory=list)
    conversaciones: List[Conversacion] = field(default_factory=list)
    tiempo_busqueda_ms: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Convierte el resultado a diccionario"""
        return {
            "total": self.total,
            "mensajes": [m.to_dict() for m in self.mensajes],
            "conversaciones": [c.to_dict() for c in self.conversaciones],
            "tiempo_busqueda_ms": self.tiempo_busqueda_ms,
        }
