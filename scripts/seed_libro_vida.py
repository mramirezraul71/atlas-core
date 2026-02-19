"""Seed the Libro de Vida with example episodes and principles."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.stdout.reconfigure(encoding="utf-8")

import requests, json

BASE = "http://127.0.0.1:8791"

episodes = [
    {
        "tarea": "Abrir puerta pesada del laboratorio",
        "tipo_tarea": "apertura_puerta",
        "plan_ejecutado": ["Detectar tipo de puerta", "Calcular fuerza necesaria", "Aplicar fuerza gradual", "Verificar espacio libre", "Mantener puerta abierta"],
        "exito": False,
        "errores": ["Fuerza insuficiente en primer intento", "Puerta reboto al no mantener presion"],
        "correcciones": ["Aumentar fuerza gradualmente", "Mantener presion constante hasta apertura completa"],
        "lecciones": "Puertas pesadas requieren fuerza sostenida. No soltar hasta confirmar apertura de 90 grados.",
        "reglas_numericas": {"fuerza_puerta_pesada_N": 40, "angulo_apertura_seguro_grados": 90},
        "contexto_entorno": "Laboratorio, puerta cortafuegos metalica",
    },
    {
        "tarea": "Subir escaleras con paquete de 5kg",
        "tipo_tarea": "escaleras",
        "plan_ejecutado": ["Verificar estabilidad", "Ajustar centro de gravedad", "Subir escalon por escalon", "Verificar agarre en cada escalon"],
        "exito": True,
        "lecciones": "Subir escaleras con carga: bajar velocidad 50 porciento, verificar agarre cada 3 escalones.",
        "metricas": {"tiempo_s": 32, "escalones": 12, "peso_kg": 5},
        "reglas_numericas": {"peso_max_escaleras_kg": 8, "velocidad_reduccion_pct": 50, "verificar_agarre_cada_escalones": 3},
        "contexto_entorno": "Edificio oficinas, escalera estandar",
    },
    {
        "tarea": "Interactuar con nino en area de juegos para entregar pelota",
        "tipo_tarea": "interaccion_humana",
        "plan_ejecutado": ["Detectar presencia de menores", "Reducir velocidad a minima", "Anunciar presencia verbalmente", "Acercar lentamente", "Ofrecer objeto a distancia segura", "Soltar suavemente"],
        "exito": True,
        "lecciones": "Con ninos: velocidad maxima 0.2 m/s, distancia minima 1m, siempre anunciar presencia.",
        "reglas_numericas": {"velocidad_max_ninos_ms": 0.2, "distancia_min_ninos_m": 1.0, "fuerza_entrega_max_N": 5},
        "feedback_humano": "Muy bien, el nino no se asusto",
        "contexto_entorno": "Area de juegos interior, 3 ninos presentes",
    },
    {
        "tarea": "Patrullar zona de almacen nocturno",
        "tipo_tarea": "patrulla",
        "plan_ejecutado": ["Activar sensores nocturnos", "Seguir ruta predefinida", "Escanear anomalias", "Reportar estado cada punto de control"],
        "exito": True,
        "lecciones": "Patrullas nocturnas requieren sensores IR activos. Reportar en cada checkpoint reduce falsos positivos.",
        "metricas": {"tiempo_total_min": 25, "checkpoints": 8, "anomalias_detectadas": 0},
        "contexto_entorno": "Almacen industrial, sin iluminacion, temperatura 15C",
    },
    {
        "tarea": "Limpiar derrame de liquido en pasillo principal",
        "tipo_tarea": "limpieza",
        "plan_ejecutado": ["Detectar extension del derrame", "Senalizar zona", "Aplicar material absorbente", "Recoger residuos", "Verificar piso seco"],
        "exito": True,
        "lecciones": "Siempre senalizar antes de limpiar. Verificar que no haya riesgo de resbalon residual.",
        "metricas": {"tiempo_min": 8, "area_m2": 2.5},
        "contexto_entorno": "Pasillo principal, trafico medio de personas",
    },
]

print("=== Registrando episodios ===")
for ep in episodes:
    r = requests.post(f"{BASE}/api/libro-vida/registrar-resultado", json=ep, timeout=15)
    d = r.json()
    status = "OK" if d.get("ok") else "FAIL"
    print(f"  {status}: {ep['tarea'][:50]} -> {d.get('episodio_id', d.get('error','?'))}")

principios = [
    {"categoria": "seguridad", "principio": "Cerca de humanos, reducir velocidad al 50% y aumentar distancia minima a 1m", "confianza": 0.9},
    {"categoria": "seguridad", "principio": "Con ninos presentes, velocidad maxima 0.2 m/s y anunciar presencia", "confianza": 0.95},
    {"categoria": "manipulacion", "principio": "Objetos fragiles requieren agarre con fuerza limitada (max 15N) y verificacion continua", "confianza": 0.85},
    {"categoria": "navegacion", "principio": "En pasillos estrechos (<1.5m), reducir velocidad a 0.3 m/s y mantener 30cm de paredes", "confianza": 0.8},
    {"categoria": "escaleras", "principio": "Subir/bajar con carga: reducir velocidad 50%, verificar agarre cada 3 escalones, peso max 8kg", "confianza": 0.85},
    {"categoria": "puertas", "principio": "Puertas pesadas requieren fuerza sostenida y no soltar hasta apertura de 90 grados", "confianza": 0.8},
    {"categoria": "limpieza", "principio": "Siempre senalizar zona antes de limpiar y verificar que no quede riesgo de resbalon", "confianza": 0.75},
]

print("\n=== Registrando principios ===")
for p in principios:
    r = requests.post(f"{BASE}/api/libro-vida/principio", json=p, timeout=10)
    d = r.json()
    print(f"  [{p['categoria']}]: {d.get('ok')}")

print("\n=== STATUS FINAL ===")
r = requests.get(f"{BASE}/api/libro-vida/status", timeout=10)
print(json.dumps(r.json(), indent=2, ensure_ascii=False))
