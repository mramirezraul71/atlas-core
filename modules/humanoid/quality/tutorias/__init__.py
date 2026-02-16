"""
ATLAS Quality - Sistema de Tutorías y Visitas de Especialistas
===============================================================

Módulo para gestionar:
- Visitas de especialistas
- Recomendaciones técnicas
- Informes de tutoría con firma digital
- Seguimiento de mejoras
- Evaluaciones del sistema

Cada especialista que instruya a ATLAS debe dejar constancia
de sus instrucciones, evaluaciones y recomendaciones.
"""

from .models import (
    Especialista,
    Visita,
    Informe,
    Recomendacion,
    Evaluacion,
    SeguimientoMejora
)
from .manager import TutoriasManager
from .reports import ReportGenerator

__all__ = [
    'Especialista',
    'Visita', 
    'Informe',
    'Recomendacion',
    'Evaluacion',
    'SeguimientoMejora',
    'TutoriasManager',
    'ReportGenerator'
]
