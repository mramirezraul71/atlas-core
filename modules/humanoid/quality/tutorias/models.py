"""
Modelos de datos para el sistema de Tutorías y Visitas
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional, Dict, Any
from enum import Enum
import hashlib
import json
import uuid


class TipoVisita(Enum):
    TUTORIA = "tutoria"
    REVISION = "revision"
    AUDITORIA = "auditoria"
    CAPACITACION = "capacitacion"
    MANTENIMIENTO = "mantenimiento"
    EMERGENCIA = "emergencia"
    SEGUIMIENTO = "seguimiento"


class NivelEvaluacion(Enum):
    EXCELENTE = 5
    BUENO = 4
    ACEPTABLE = 3
    MEJORABLE = 2
    CRITICO = 1


class EstadoRecomendacion(Enum):
    PENDIENTE = "pendiente"
    EN_PROGRESO = "en_progreso"
    COMPLETADA = "completada"
    DESCARTADA = "descartada"
    POSTERGADA = "postergada"


class PrioridadRecomendacion(Enum):
    CRITICA = 1
    ALTA = 2
    MEDIA = 3
    BAJA = 4
    OPCIONAL = 5


@dataclass
class Especialista:
    """Especialista que realiza visitas y tutorías"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    nombre: str = ""
    rol: str = ""  # Arquitecto, Desarrollador, QA, DevOps, etc.
    especialidad: str = ""  # Vision, NLP, Robotics, etc.
    email: str = ""
    firma_digital: str = ""  # Hash de verificación
    fecha_registro: datetime = field(default_factory=datetime.now)
    visitas_realizadas: int = 0
    
    def generar_firma(self) -> str:
        """Genera firma digital única del especialista"""
        data = f"{self.id}:{self.nombre}:{self.email}:{datetime.now().isoformat()}"
        self.firma_digital = hashlib.sha256(data.encode()).hexdigest()[:16]
        return self.firma_digital
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "nombre": self.nombre,
            "rol": self.rol,
            "especialidad": self.especialidad,
            "email": self.email,
            "firma_digital": self.firma_digital,
            "fecha_registro": self.fecha_registro.isoformat(),
            "visitas_realizadas": self.visitas_realizadas
        }


@dataclass
class Evaluacion:
    """Evaluación de un aspecto del sistema"""
    aspecto: str = ""  # Ej: "Vision", "Respuesta", "Código"
    nivel: NivelEvaluacion = NivelEvaluacion.ACEPTABLE
    puntuacion: float = 3.0  # 1-5
    comentario: str = ""
    metricas: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "aspecto": self.aspecto,
            "nivel": self.nivel.name,
            "puntuacion": self.puntuacion,
            "comentario": self.comentario,
            "metricas": self.metricas
        }


@dataclass
class Recomendacion:
    """Recomendación técnica de un especialista"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    titulo: str = ""
    descripcion: str = ""
    modulo_afectado: str = ""
    prioridad: PrioridadRecomendacion = PrioridadRecomendacion.MEDIA
    estado: EstadoRecomendacion = EstadoRecomendacion.PENDIENTE
    fecha_creacion: datetime = field(default_factory=datetime.now)
    fecha_limite: Optional[datetime] = None
    pasos_implementacion: List[str] = field(default_factory=list)
    recursos_necesarios: List[str] = field(default_factory=list)
    especialista_id: str = ""
    notas_seguimiento: List[str] = field(default_factory=list)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "titulo": self.titulo,
            "descripcion": self.descripcion,
            "modulo_afectado": self.modulo_afectado,
            "prioridad": self.prioridad.name,
            "estado": self.estado.name,
            "fecha_creacion": self.fecha_creacion.isoformat(),
            "fecha_limite": self.fecha_limite.isoformat() if self.fecha_limite else None,
            "pasos_implementacion": self.pasos_implementacion,
            "recursos_necesarios": self.recursos_necesarios,
            "especialista_id": self.especialista_id,
            "notas_seguimiento": self.notas_seguimiento
        }


@dataclass
class Informe:
    """Informe detallado de una visita/tutoría"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    titulo: str = ""
    resumen: str = ""
    contenido: str = ""
    objetivos_cumplidos: List[str] = field(default_factory=list)
    objetivos_pendientes: List[str] = field(default_factory=list)
    evaluaciones: List[Evaluacion] = field(default_factory=list)
    recomendaciones: List[Recomendacion] = field(default_factory=list)
    archivos_adjuntos: List[str] = field(default_factory=list)
    codigo_revisado: List[str] = field(default_factory=list)  # Paths de archivos
    metricas_capturadas: Dict[str, Any] = field(default_factory=dict)
    proximos_pasos: List[str] = field(default_factory=list)
    
    # Firma
    firmado: bool = False
    firma_especialista: str = ""
    fecha_firma: Optional[datetime] = None
    hash_verificacion: str = ""
    
    def calcular_puntuacion_global(self) -> float:
        """Calcula la puntuación promedio de todas las evaluaciones"""
        if not self.evaluaciones:
            return 0.0
        return sum(e.puntuacion for e in self.evaluaciones) / len(self.evaluaciones)
    
    def firmar(self, especialista: Especialista) -> bool:
        """Firma el informe con la firma del especialista"""
        if self.firmado:
            return False
        
        # Generar hash de verificación
        content = json.dumps({
            "id": self.id,
            "titulo": self.titulo,
            "contenido": self.contenido,
            "especialista": especialista.id,
            "timestamp": datetime.now().isoformat()
        }, sort_keys=True)
        
        self.hash_verificacion = hashlib.sha256(content.encode()).hexdigest()
        self.firma_especialista = especialista.firma_digital or especialista.generar_firma()
        self.fecha_firma = datetime.now()
        self.firmado = True
        return True
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "titulo": self.titulo,
            "resumen": self.resumen,
            "contenido": self.contenido,
            "objetivos_cumplidos": self.objetivos_cumplidos,
            "objetivos_pendientes": self.objetivos_pendientes,
            "evaluaciones": [e.to_dict() for e in self.evaluaciones],
            "recomendaciones": [r.to_dict() for r in self.recomendaciones],
            "archivos_adjuntos": self.archivos_adjuntos,
            "codigo_revisado": self.codigo_revisado,
            "metricas_capturadas": self.metricas_capturadas,
            "proximos_pasos": self.proximos_pasos,
            "puntuacion_global": self.calcular_puntuacion_global(),
            "firmado": self.firmado,
            "firma_especialista": self.firma_especialista,
            "fecha_firma": self.fecha_firma.isoformat() if self.fecha_firma else None,
            "hash_verificacion": self.hash_verificacion
        }


@dataclass
class Visita:
    """Registro de una visita de especialista"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    tipo: TipoVisita = TipoVisita.TUTORIA
    especialista: Optional[Especialista] = None
    fecha_inicio: datetime = field(default_factory=datetime.now)
    fecha_fin: Optional[datetime] = None
    duracion_minutos: int = 0
    
    # Contexto
    motivo: str = ""
    modulos_revisados: List[str] = field(default_factory=list)
    objetivos: List[str] = field(default_factory=list)
    
    # Resultado
    informe: Optional[Informe] = None
    completada: bool = False
    
    # Metadatos
    version_atlas: str = ""
    ambiente: str = "desarrollo"  # desarrollo, staging, produccion
    notas: str = ""
    
    def finalizar(self) -> None:
        """Marca la visita como completada"""
        self.fecha_fin = datetime.now()
        if self.fecha_inicio:
            delta = self.fecha_fin - self.fecha_inicio
            self.duracion_minutos = int(delta.total_seconds() / 60)
        self.completada = True
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "tipo": self.tipo.value,
            "especialista": self.especialista.to_dict() if self.especialista else None,
            "fecha_inicio": self.fecha_inicio.isoformat(),
            "fecha_fin": self.fecha_fin.isoformat() if self.fecha_fin else None,
            "duracion_minutos": self.duracion_minutos,
            "motivo": self.motivo,
            "modulos_revisados": self.modulos_revisados,
            "objetivos": self.objetivos,
            "informe": self.informe.to_dict() if self.informe else None,
            "completada": self.completada,
            "version_atlas": self.version_atlas,
            "ambiente": self.ambiente,
            "notas": self.notas
        }


@dataclass
class SeguimientoMejora:
    """Seguimiento de una mejora recomendada"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    recomendacion_id: str = ""
    titulo: str = ""
    descripcion: str = ""
    fecha_inicio: datetime = field(default_factory=datetime.now)
    fecha_objetivo: Optional[datetime] = None
    porcentaje_avance: int = 0
    hitos: List[Dict[str, Any]] = field(default_factory=list)
    responsable: str = ""
    ultima_actualizacion: datetime = field(default_factory=datetime.now)
    completado: bool = False
    
    def agregar_hito(self, descripcion: str, completado: bool = False) -> None:
        """Agrega un hito al seguimiento"""
        self.hitos.append({
            "descripcion": descripcion,
            "completado": completado,
            "fecha": datetime.now().isoformat()
        })
        self.ultima_actualizacion = datetime.now()
        self._recalcular_avance()
    
    def _recalcular_avance(self) -> None:
        """Recalcula el porcentaje de avance basado en hitos"""
        if not self.hitos:
            self.porcentaje_avance = 0
            return
        completados = sum(1 for h in self.hitos if h.get("completado", False))
        self.porcentaje_avance = int((completados / len(self.hitos)) * 100)
        if self.porcentaje_avance == 100:
            self.completado = True
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "recomendacion_id": self.recomendacion_id,
            "titulo": self.titulo,
            "descripcion": self.descripcion,
            "fecha_inicio": self.fecha_inicio.isoformat(),
            "fecha_objetivo": self.fecha_objetivo.isoformat() if self.fecha_objetivo else None,
            "porcentaje_avance": self.porcentaje_avance,
            "hitos": self.hitos,
            "responsable": self.responsable,
            "ultima_actualizacion": self.ultima_actualizacion.isoformat(),
            "completado": self.completado
        }
