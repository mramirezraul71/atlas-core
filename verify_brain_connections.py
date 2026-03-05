#!/usr/bin/env python3
"""
Verificación completa de conexiones cerebrales ATLAS.
Prueba que todos los módulos cognitivos estén conectados y funcionales.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from typing import Tuple


def test_module(name: str, test_fn) -> Tuple[bool, str]:
    """Ejecuta test de un módulo."""
    try:
        result = test_fn()
        return True, result
    except Exception as e:
        return False, str(e)


def verify_all():
    """Verifica todas las conexiones."""
    print("\n" + "=" * 70)
    print("  VERIFICACIÓN DE CONEXIONES CEREBRALES ATLAS")
    print("=" * 70 + "\n")

    results = {}
    total_modules = 0
    ok_modules = 0

    # ============ 1. MEDULLA (Bus de comunicación) ============
    print("🔌 MEDULLA (Bus de Comunicación)")
    print("-" * 50)

    def test_medulla():
        from modules.humanoid.medulla import MedullaAtlas, SharedState

        bus = MedullaAtlas(use_zmq=False)
        state = SharedState()
        state.write("test_key", "test_value")
        assert state.read("test_key") == "test_value"
        return "Threading bus OK, SharedState OK"

    ok, msg = test_module("medulla", test_medulla)
    print(f"  {'✅' if ok else '❌'} MedullaAtlas + SharedState: {msg}")
    results["medulla"] = ok
    total_modules += 1
    ok_modules += 1 if ok else 0

    # ============ 2. CORTEX FRONTAL ============
    print("\n🎯 CORTEX FRONTAL (Planificación)")
    print("-" * 50)

    def test_task_planner():
        from modules.humanoid.cortex.frontal import TaskPlanner

        tp = TaskPlanner()
        return "TaskPlanner initialized"

    def test_decision_maker():
        from modules.humanoid.cortex.frontal import DecisionMaker

        dm = DecisionMaker()
        return "DecisionMaker initialized"

    def test_inhibitory_control():
        from modules.humanoid.cortex.frontal import InhibitoryControl

        ic = InhibitoryControl()
        return "InhibitoryControl initialized"

    for name, fn in [
        ("TaskPlanner", test_task_planner),
        ("DecisionMaker", test_decision_maker),
        ("InhibitoryControl", test_inhibitory_control),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"frontal_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 3. CORTEX PARIETAL ============
    print("\n🗺️ CORTEX PARIETAL (Fusión Sensorial)")
    print("-" * 50)

    def test_sensory_fusion():
        from modules.humanoid.cortex.parietal import SensoryFusion

        sf = SensoryFusion()
        return "SensoryFusion initialized"

    def test_spatial_map():
        from modules.humanoid.cortex.parietal import SpatialMap

        sm = SpatialMap()
        return "SpatialMap initialized"

    def test_body_schema():
        from modules.humanoid.cortex.parietal import BodySchema

        bs = BodySchema()
        return "BodySchema initialized"

    for name, fn in [
        ("SensoryFusion", test_sensory_fusion),
        ("SpatialMap", test_spatial_map),
        ("BodySchema", test_body_schema),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"parietal_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 4. CORTEX TEMPORAL ============
    print("\n👂 CORTEX TEMPORAL (Audio y Lenguaje)")
    print("-" * 50)

    def test_audio_processor():
        from modules.humanoid.cortex.temporal import AudioProcessor

        ap = AudioProcessor()
        return "AudioProcessor initialized"

    def test_language_understanding():
        from modules.humanoid.cortex.temporal import LanguageUnderstanding

        lu = LanguageUnderstanding()
        return "LanguageUnderstanding initialized"

    def test_episodic_recall():
        from modules.humanoid.cortex.temporal import EpisodicRecall
        from modules.humanoid.hippo import HippoAPI

        hippo = HippoAPI()
        er = EpisodicRecall(hippo_api=hippo)
        return "EpisodicRecall connected to HippoAPI"

    for name, fn in [
        ("AudioProcessor", test_audio_processor),
        ("LanguageUnderstanding", test_language_understanding),
        ("EpisodicRecall", test_episodic_recall),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"temporal_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 5. CORTEX OCCIPITAL ============
    print("\n👁️ CORTEX OCCIPITAL (Visión)")
    print("-" * 50)

    def test_vision_pipeline():
        from modules.humanoid.cortex.occipital import VisionPipeline

        vp = VisionPipeline()
        return "VisionPipeline initialized"

    def test_depth_estimation():
        from modules.humanoid.cortex.occipital import DepthEstimation

        de = DepthEstimation()
        return "DepthEstimation initialized"

    def test_object_recognition():
        from modules.humanoid.cortex.occipital import ObjectRecognition

        orec = ObjectRecognition()
        return "ObjectRecognition initialized"

    for name, fn in [
        ("VisionPipeline", test_vision_pipeline),
        ("DepthEstimation", test_depth_estimation),
        ("ObjectRecognition", test_object_recognition),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"occipital_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 6. SISTEMA LÍMBICO ============
    print("\n❤️ SISTEMA LÍMBICO (Motivación)")
    print("-" * 50)

    def test_goal_manager():
        from modules.humanoid.limbic import GoalManager

        gm = GoalManager()
        return "GoalManager initialized"

    def test_reward_engine():
        from modules.humanoid.limbic import RewardEngine

        re = RewardEngine()
        return "RewardEngine initialized"

    def test_state_regulator():
        from modules.humanoid.limbic import StateRegulator

        sr = StateRegulator()
        return "StateRegulator initialized"

    for name, fn in [
        ("GoalManager", test_goal_manager),
        ("RewardEngine", test_reward_engine),
        ("StateRegulator", test_state_regulator),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"limbic_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 7. HIPOCAMPO ============
    print("\n🧩 HIPOCAMPO (Memoria)")
    print("-" * 50)

    def test_hippo_api():
        from modules.humanoid.hippo import (Consolidator, EpisodicMemory,
                                            HippoAPI, SemanticMemory)

        hippo = HippoAPI()
        em = EpisodicMemory()
        sm = SemanticMemory()
        c = Consolidator(em, sm)
        return "HippoAPI + EpisodicMemory + SemanticMemory + Consolidator"

    ok, msg = test_module("hippo", test_hippo_api)
    print(f"  {'✅' if ok else '❌'} HippoAPI: {msg}")
    results["hippo"] = ok
    total_modules += 1
    ok_modules += 1 if ok else 0

    # ============ 8. BRAINSTEM ============
    print("\n🫀 BRAINSTEM (Funciones Vitales)")
    print("-" * 50)

    def test_vitals_monitor():
        from modules.humanoid.brainstem import VitalsMonitor

        vm = VitalsMonitor()
        return "VitalsMonitor initialized"

    def test_safety_policy():
        from modules.humanoid.brainstem import SafetyPolicy

        sp = SafetyPolicy()
        return "SafetyPolicy initialized"

    def test_global_state():
        from modules.humanoid.brainstem import GlobalState

        gs = GlobalState()
        return "GlobalState initialized"

    def test_watchdog():
        from modules.humanoid.brainstem import Watchdog

        wd = Watchdog()
        return "Watchdog initialized"

    for name, fn in [
        ("VitalsMonitor", test_vitals_monitor),
        ("SafetyPolicy", test_safety_policy),
        ("GlobalState", test_global_state),
        ("Watchdog", test_watchdog),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"brainstem_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 9. GANGLIOS BASALES ============
    print("\n⚡ GANGLIOS BASALES (Selección de Acciones)")
    print("-" * 50)

    def test_action_selector():
        from modules.humanoid.basal import ActionSelector

        asel = ActionSelector()
        return "ActionSelector initialized"

    def test_inhibitor():
        from modules.humanoid.basal import Inhibitor

        inh = Inhibitor()
        return "Inhibitor initialized"

    for name, fn in [
        ("ActionSelector", test_action_selector),
        ("Inhibitor", test_inhibitor),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"basal_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 10. MOTOR CONTROL ============
    print("\n🦾 MOTOR CONTROL (Movimiento)")
    print("-" * 50)

    def test_trajectory_planner():
        from modules.humanoid.motor import TrajectoryPlanner

        tp = TrajectoryPlanner()
        return "TrajectoryPlanner initialized"

    def test_motor_controller():
        from modules.humanoid.motor import MotorController

        mc = MotorController()
        return "MotorController initialized"

    def test_motor_interface():
        from modules.humanoid.motor import (MotorController, MotorInterface,
                                            TrajectoryPlanner)

        tp = TrajectoryPlanner()
        mc = MotorController()
        mi = MotorInterface(tp, mc)
        return "MotorInterface connected to Planner+Controller"

    for name, fn in [
        ("TrajectoryPlanner", test_trajectory_planner),
        ("MotorController", test_motor_controller),
        ("MotorInterface", test_motor_interface),
    ]:
        ok, msg = test_module(name, fn)
        print(f"  {'✅' if ok else '❌'} {name}: {msg}")
        results[f"motor_{name}"] = ok
        total_modules += 1
        ok_modules += 1 if ok else 0

    # ============ 11. LEARNING ============
    print("\n📚 LEARNING (Aprendizaje)")
    print("-" * 50)

    def test_learning_api():
        from modules.humanoid.learning import (DemonstrationLearning,
                                               LearningAPI,
                                               NaturalLanguageFeedback,
                                               ReinforcementLearning)

        dl = DemonstrationLearning()
        rl = ReinforcementLearning()
        nlf = NaturalLanguageFeedback()
        api = LearningAPI()
        return "LearningAPI + Demonstration + RL + NLFeedback"

    ok, msg = test_module("learning", test_learning_api)
    print(f"  {'✅' if ok else '❌'} LearningAPI: {msg}")
    results["learning"] = ok
    total_modules += 1
    ok_modules += 1 if ok else 0

    # ============ 12. COGNITIVE API ============
    print("\n🌐 COGNITIVE API (Dashboard)")
    print("-" * 50)

    def test_cognitive_api():
        from modules.humanoid.cognitive.api import cognitive_status

        status = cognitive_status()
        return "API router + status endpoint OK"

    ok, msg = test_module("cognitive_api", test_cognitive_api)
    print(f"  {'✅' if ok else '❌'} Cognitive API: {msg}")
    results["cognitive_api"] = ok
    total_modules += 1
    ok_modules += 1 if ok else 0

    # ============ 13. INTEGRACIÓN COMPLETA ============
    print("\n🧠 SISTEMA COGNITIVO COMPLETO")
    print("-" * 50)

    def test_full_system():
        from modules.humanoid import create_cognitive_system

        system = create_cognitive_system()

        # Verificar todas las conexiones
        assert "medulla" in system, "Medulla no conectada"
        assert "cortex" in system, "Cortex no conectado"
        assert "frontal" in system["cortex"], "Cortex frontal no conectado"
        assert "parietal" in system["cortex"], "Cortex parietal no conectado"
        assert "temporal" in system["cortex"], "Cortex temporal no conectado"
        assert "occipital" in system["cortex"], "Cortex occipital no conectado"
        assert "limbic" in system, "Sistema límbico no conectado"
        assert "hippo" in system, "Hipocampo no conectado"
        assert "brainstem" in system, "Brainstem no conectado"
        assert "basal" in system, "Ganglios basales no conectados"
        assert "motor" in system, "Motor control no conectado"
        assert "learning" in system, "Learning no conectado"

        # Contar módulos
        count = 0
        count += 1  # medulla
        count += len(system["cortex"]["frontal"])
        count += len(system["cortex"]["parietal"])
        count += len(system["cortex"]["temporal"])
        count += len(system["cortex"]["occipital"])
        count += len(system["limbic"])
        count += 1  # hippo
        count += len(system["brainstem"])
        count += len(system["basal"])
        count += len(system["motor"])
        count += 1  # learning

        return f"{count} módulos conectados al cerebro"

    ok, msg = test_module("full_system", test_full_system)
    print(f"  {'✅' if ok else '❌'} Sistema completo: {msg}")
    results["full_system"] = ok
    total_modules += 1
    ok_modules += 1 if ok else 0

    # ============ RESUMEN ============
    print("\n" + "=" * 70)
    print("  RESUMEN DE VERIFICACIÓN")
    print("=" * 70)

    failed = [k for k, v in results.items() if not v]

    print(f"\n  Total módulos verificados: {total_modules}")
    print(f"  Módulos OK: {ok_modules}")
    print(f"  Módulos con error: {len(failed)}")

    if failed:
        print(f"\n  ❌ Módulos fallidos: {', '.join(failed)}")

    percentage = (ok_modules / total_modules) * 100

    print(f"\n  Conexión cerebral: {percentage:.1f}%")

    if percentage == 100:
        print("\n  🟢 TODOS LOS MÓDULOS CONECTADOS AL CEREBRO")
    elif percentage >= 80:
        print("\n  🟡 SISTEMA OPERATIVO CON MÓDULOS PARCIALES")
    else:
        print("\n  🔴 SISTEMA CON PROBLEMAS DE CONEXIÓN")

    print("\n" + "=" * 70 + "\n")

    return percentage == 100


if __name__ == "__main__":
    success = verify_all()
    sys.exit(0 if success else 1)
