"""
Ejemplos de uso del sistema de Tutorías y Visitas
=================================================

Este archivo muestra cómo usar el sistema completo de tutorías,
incluyendo el registro de especialistas, visitas, informes y seguimientos.
"""

from datetime import datetime, timedelta
from .models import (
    Especialista, Visita, Informe, Evaluacion, Recomendacion,
    TipoVisita, NivelEvaluacion, PrioridadRecomendacion
)
from .manager import TutoriasManager
from .reports import ReportGenerator


def ejemplo_flujo_completo():
    """
    Ejemplo de flujo completo de una tutoría:
    1. Registrar especialista
    2. Iniciar visita
    3. Realizar evaluaciones
    4. Crear recomendaciones
    5. Finalizar con informe firmado
    6. Generar reportes
    """
    
    # Inicializar manager
    manager = TutoriasManager()
    reporter = ReportGenerator()
    
    # 1. Registrar especialista
    print("=" * 50)
    print("1. REGISTRANDO ESPECIALISTA")
    print("=" * 50)
    
    especialista = Especialista(
        nombre="Dr. Carlos Mendoza",
        rol="Arquitecto de IA",
        especialidad="Sistemas de Visión y NLP",
        email="carlos.mendoza@atlas.ai"
    )
    especialista.generar_firma()
    
    esp_id = manager.registrar_especialista(especialista)
    print(f"✓ Especialista registrado: {especialista.nombre}")
    print(f"  ID: {esp_id}")
    print(f"  Firma Digital: {especialista.firma_digital}")
    
    # 2. Iniciar visita
    print("\n" + "=" * 50)
    print("2. INICIANDO VISITA DE TUTORÍA")
    print("=" * 50)
    
    visita = manager.iniciar_visita(
        especialista=especialista,
        tipo=TipoVisita.TUTORIA,
        motivo="Revisión trimestral del sistema de visión",
        modulos=["vision_engine", "face_detection", "ocr_module"],
        objetivos=[
            "Evaluar rendimiento del sistema de visión",
            "Revisar integración con cerebro ATLAS",
            "Proponer mejoras de eficiencia"
        ]
    )
    
    print(f"✓ Visita iniciada: {visita.id}")
    print(f"  Tipo: {visita.tipo.value}")
    print(f"  Motivo: {visita.motivo}")
    print(f"  Módulos: {', '.join(visita.modulos_revisados)}")
    
    # Simular trabajo durante la visita...
    print("\n  [Realizando evaluación...]")
    
    # 3. Crear evaluaciones
    print("\n" + "=" * 50)
    print("3. REALIZANDO EVALUACIONES")
    print("=" * 50)
    
    evaluaciones = [
        Evaluacion(
            aspecto="Precisión de detección facial",
            nivel=NivelEvaluacion.BUENO,
            puntuacion=4.2,
            comentario="Buena precisión en condiciones óptimas de luz",
            metricas={"precision": 0.94, "recall": 0.91}
        ),
        Evaluacion(
            aspecto="Rendimiento OCR",
            nivel=NivelEvaluacion.ACEPTABLE,
            puntuacion=3.5,
            comentario="Requiere optimización para textos pequeños",
            metricas={"accuracy": 0.87, "latency_ms": 120}
        ),
        Evaluacion(
            aspecto="Integración con cerebro",
            nivel=NivelEvaluacion.EXCELENTE,
            puntuacion=4.8,
            comentario="Excelente comunicación y respuesta",
            metricas={"response_time_ms": 45}
        ),
        Evaluacion(
            aspecto="Manejo de errores",
            nivel=NivelEvaluacion.MEJORABLE,
            puntuacion=2.8,
            comentario="Falta logging detallado en casos de fallo",
            metricas={"error_recovery_rate": 0.72}
        )
    ]
    
    for ev in evaluaciones:
        print(f"  • {ev.aspecto}: {ev.puntuacion}/5 ({ev.nivel.name})")
    
    # 4. Crear recomendaciones
    print("\n" + "=" * 50)
    print("4. GENERANDO RECOMENDACIONES")
    print("=" * 50)
    
    recomendaciones = [
        Recomendacion(
            titulo="Implementar modelo de visión nocturna",
            descripcion="Agregar capacidad de procesamiento en condiciones de baja luz usando técnicas de enhancement",
            modulo_afectado="vision_engine",
            prioridad=PrioridadRecomendacion.ALTA,
            pasos_implementacion=[
                "Investigar modelos de enhancement de imagen",
                "Implementar preprocesamiento adaptativo",
                "Entrenar con dataset nocturno",
                "Integrar con pipeline existente",
                "Pruebas de rendimiento"
            ]
        ),
        Recomendacion(
            titulo="Optimizar latencia de OCR",
            descripcion="Reducir el tiempo de respuesta del módulo OCR mediante caché y procesamiento paralelo",
            modulo_afectado="ocr_module",
            prioridad=PrioridadRecomendacion.MEDIA,
            pasos_implementacion=[
                "Implementar sistema de caché para textos frecuentes",
                "Paralelizar procesamiento de regiones",
                "Optimizar modelo con quantization"
            ]
        ),
        Recomendacion(
            titulo="Mejorar sistema de logging",
            descripcion="Implementar logging estructurado con niveles y contexto para facilitar debugging",
            modulo_afectado="vision_engine",
            prioridad=PrioridadRecomendacion.CRITICA,
            pasos_implementacion=[
                "Definir estándares de logging",
                "Implementar decoradores de logging",
                "Agregar contexto a errores",
                "Integrar con bitácora central"
            ]
        )
    ]
    
    for rec in recomendaciones:
        print(f"  [{rec.prioridad.name}] {rec.titulo}")
        print(f"     Módulo: {rec.modulo_afectado}")
        print(f"     Pasos: {len(rec.pasos_implementacion)}")
    
    # 5. Crear y firmar informe
    print("\n" + "=" * 50)
    print("5. CREANDO INFORME FIRMADO")
    print("=" * 50)
    
    informe = Informe(
        titulo="Auditoría del Sistema de Visión Q1-2026",
        resumen="Evaluación completa del sistema de visión de ATLAS con enfoque en rendimiento, precisión e integración.",
        contenido="""
Durante esta visita de tutoría se realizó una evaluación exhaustiva del sistema de visión de ATLAS.

## Hallazgos Principales

1. **Detección Facial**: El sistema muestra buen rendimiento en condiciones de iluminación estándar,
   con una precisión del 94%. Sin embargo, se detectaron áreas de mejora para escenarios con baja luz.

2. **OCR**: El módulo funciona correctamente pero presenta latencia elevada (120ms promedio).
   Se recomienda optimización mediante caché y procesamiento paralelo.

3. **Integración con Cerebro**: Excelente comunicación bidireccional. El tiempo de respuesta
   de 45ms es óptimo para aplicaciones en tiempo real.

4. **Manejo de Errores**: Área crítica que requiere atención. El sistema necesita mejor
   logging y recuperación de errores.

## Conclusión

El sistema de visión de ATLAS está funcionalmente completo pero requiere optimizaciones
específicas para alcanzar nivel de producción enterprise.
        """,
        objetivos_cumplidos=[
            "Evaluar rendimiento del sistema de visión",
            "Revisar integración con cerebro ATLAS"
        ],
        objetivos_pendientes=[
            "Proponer mejoras de eficiencia (parcial - ver recomendaciones)"
        ],
        evaluaciones=evaluaciones,
        recomendaciones=recomendaciones,
        proximos_pasos=[
            "Implementar recomendación crítica de logging",
            "Planificar sprint de optimización de OCR",
            "Agendar siguiente revisión en 30 días"
        ]
    )
    
    # Finalizar visita (esto firma el informe automáticamente)
    visita_final = manager.finalizar_visita(informe)
    
    print(f"✓ Visita finalizada")
    print(f"  Duración: {visita_final.duracion_minutos} minutos")
    print(f"  Informe firmado: {informe.firmado}")
    print(f"  Firma: {informe.firma_especialista}")
    print(f"  Hash: {informe.hash_verificacion}")
    print(f"  Puntuación Global: {informe.calcular_puntuacion_global():.2f}/5")
    
    # 6. Generar reportes
    print("\n" + "=" * 50)
    print("6. GENERANDO REPORTES")
    print("=" * 50)
    
    # Generar en múltiples formatos
    for formato in ["markdown", "html", "json", "txt"]:
        path = reporter.generar_informe_visita(visita_final, formato)
        print(f"  ✓ Reporte {formato.upper()}: {path}")
    
    # 7. Crear seguimientos
    print("\n" + "=" * 50)
    print("7. CREANDO SEGUIMIENTOS DE MEJORA")
    print("=" * 50)
    
    for rec in recomendaciones:
        seguimiento = manager.crear_seguimiento(rec, "ATLAS Sistema")
        print(f"  ✓ Seguimiento creado: {seguimiento.titulo}")
        print(f"     ID: {seguimiento.id}")
        print(f"     Hitos: {len(seguimiento.hitos)}")
    
    # 8. Obtener estadísticas
    print("\n" + "=" * 50)
    print("8. ESTADÍSTICAS DEL SISTEMA")
    print("=" * 50)
    
    stats = manager.obtener_estadisticas()
    print(f"  Total Especialistas: {stats.get('total_especialistas', 0)}")
    print(f"  Total Visitas: {stats.get('total_visitas', 0)}")
    print(f"  Informes Firmados: {stats.get('informes_firmados', 0)}")
    print(f"  Puntuación Promedio: {stats.get('puntuacion_promedio', 0)}")
    print(f"  Seguimientos Activos: {stats.get('seguimientos_activos', 0)}")
    
    # Generar dashboard
    seguimientos = manager.listar_seguimientos(solo_activos=True)
    dashboard_path = reporter.generar_dashboard_estado(stats, seguimientos)
    print(f"\n  ✓ Dashboard generado: {dashboard_path}")
    
    print("\n" + "=" * 50)
    print("FLUJO COMPLETO FINALIZADO")
    print("=" * 50)
    
    return visita_final


def ejemplo_consultas():
    """Ejemplo de consultas al sistema"""
    
    manager = TutoriasManager()
    
    print("CONSULTAS AL SISTEMA DE TUTORÍAS")
    print("=" * 50)
    
    # Listar especialistas
    especialistas = manager.listar_especialistas()
    print(f"\nEspecialistas registrados: {len(especialistas)}")
    for esp in especialistas:
        print(f"  - {esp.nombre} ({esp.rol}) - {esp.visitas_realizadas} visitas")
    
    # Listar visitas recientes
    visitas = manager.listar_visitas(limite=5)
    print(f"\nÚltimas 5 visitas:")
    for v in visitas:
        print(f"  - [{v.tipo.value}] {v.motivo} - {'✓' if v.completada else '⏳'}")
    
    # Listar recomendaciones pendientes
    from .models import EstadoRecomendacion
    recs_pendientes = manager.listar_recomendaciones(
        estado=EstadoRecomendacion.PENDIENTE
    )
    print(f"\nRecomendaciones pendientes: {len(recs_pendientes)}")
    for rec in recs_pendientes[:5]:
        print(f"  - [{rec.prioridad.name}] {rec.titulo}")
    
    # Seguimientos activos
    seguimientos = manager.listar_seguimientos(solo_activos=True)
    print(f"\nSeguimientos activos: {len(seguimientos)}")
    for seg in seguimientos[:5]:
        print(f"  - {seg.titulo}: {seg.porcentaje_avance}% completado")


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("  SISTEMA DE TUTORÍAS Y VISITAS - ATLAS")
    print("  Ejemplo de Flujo Completo")
    print("=" * 60 + "\n")
    
    ejemplo_flujo_completo()
    
    print("\n" + "-" * 60 + "\n")
    
    ejemplo_consultas()
