#!/usr/bin/env python3
"""
Test script for Atlas Cognitive System.

Verifies that all modules can be imported and instantiated.
"""
import sys
import os

# Add project root to path
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def test_imports():
    """Test all cognitive module imports."""
    print("=" * 60)
    print("ATLAS COGNITIVE SYSTEM - Import Test")
    print("=" * 60)
    
    results = {}
    
    # Test Medulla
    try:
        from modules.humanoid.medulla import MedullaAtlas, SharedState
        medulla = MedullaAtlas(use_zmq=False)
        results["Medulla"] = "OK"
        print("  [OK] Medulla Atlas (bus de comunicacion)")
    except Exception as e:
        results["Medulla"] = f"FAIL: {e}"
        print(f"  [FAIL] Medulla: {e}")
    
    # Test Cortex Frontal
    try:
        from modules.humanoid.cortex.frontal import TaskPlanner, DecisionMaker, InhibitoryControl
        planner = TaskPlanner()
        results["Cortex.Frontal"] = "OK"
        print("  [OK] Cortex.Frontal (planificacion y decision)")
    except Exception as e:
        results["Cortex.Frontal"] = f"FAIL: {e}"
        print(f"  [FAIL] Cortex.Frontal: {e}")
    
    # Test Cortex Parietal
    try:
        from modules.humanoid.cortex.parietal import SensoryFusion, SpatialMap, BodySchema
        fusion = SensoryFusion()
        results["Cortex.Parietal"] = "OK"
        print("  [OK] Cortex.Parietal (fusion sensorial)")
    except Exception as e:
        results["Cortex.Parietal"] = f"FAIL: {e}"
        print(f"  [FAIL] Cortex.Parietal: {e}")
    
    # Test Cortex Temporal
    try:
        from modules.humanoid.cortex.temporal import AudioProcessor, LanguageUnderstanding, EpisodicRecall
        nlu = LanguageUnderstanding()
        results["Cortex.Temporal"] = "OK"
        print("  [OK] Cortex.Temporal (audio y lenguaje)")
    except Exception as e:
        results["Cortex.Temporal"] = f"FAIL: {e}"
        print(f"  [FAIL] Cortex.Temporal: {e}")
    
    # Test Cortex Occipital
    try:
        from modules.humanoid.cortex.occipital import VisionPipeline, DepthEstimation, ObjectRecognition
        vision = VisionPipeline()
        depth = DepthEstimation()
        results["Cortex.Occipital"] = "OK"
        print("  [OK] Cortex.Occipital (vision y profundidad)")
    except Exception as e:
        results["Cortex.Occipital"] = f"FAIL: {e}"
        print(f"  [FAIL] Cortex.Occipital: {e}")
    
    # Test Limbic
    try:
        from modules.humanoid.limbic import GoalManager, RewardEngine, StateRegulator
        goals = GoalManager()
        results["Limbic"] = "OK"
        print("  [OK] Limbic (sistema motivacional)")
    except Exception as e:
        results["Limbic"] = f"FAIL: {e}"
        print(f"  [FAIL] Limbic: {e}")
    
    # Test Hippo
    try:
        from modules.humanoid.hippo import HippoAPI, EpisodicMemory, SemanticMemory
        hippo = HippoAPI()
        results["Hippo"] = "OK"
        print("  [OK] Hippo (memoria episodica y semantica)")
    except Exception as e:
        results["Hippo"] = f"FAIL: {e}"
        print(f"  [FAIL] Hippo: {e}")
    
    # Test Brainstem
    try:
        from modules.humanoid.brainstem import VitalsMonitor, SafetyPolicy, GlobalState, Watchdog
        vitals = VitalsMonitor()
        results["Brainstem"] = "OK"
        print("  [OK] Brainstem (monitoreo vital)")
    except Exception as e:
        results["Brainstem"] = f"FAIL: {e}"
        print(f"  [FAIL] Brainstem: {e}")
    
    # Test Basal
    try:
        from modules.humanoid.basal import ActionSelector, Inhibitor
        selector = ActionSelector()
        inhibitor = Inhibitor()
        results["Basal"] = "OK"
        print("  [OK] Basal (seleccion de acciones)")
    except Exception as e:
        results["Basal"] = f"FAIL: {e}"
        print(f"  [FAIL] Basal: {e}")
    
    # Test Motor
    try:
        from modules.humanoid.motor import TrajectoryPlanner, MotorController, MotorInterface
        traj = TrajectoryPlanner()
        results["Motor"] = "OK"
        print("  [OK] Motor (control de movimiento)")
    except Exception as e:
        results["Motor"] = f"FAIL: {e}"
        print(f"  [FAIL] Motor: {e}")
    
    # Test Learning
    try:
        from modules.humanoid.learning import LearningAPI, DemonstrationLearning, ReinforcementLearning
        learning = LearningAPI()
        results["Learning"] = "OK"
        print("  [OK] Learning (LfD, RL, NL feedback)")
    except Exception as e:
        results["Learning"] = f"FAIL: {e}"
        print(f"  [FAIL] Learning: {e}")
    
    # Summary
    print("\n" + "=" * 60)
    print("RESUMEN")
    print("=" * 60)
    
    passed = sum(1 for v in results.values() if v == "OK")
    total = len(results)
    
    print(f"\n  Modulos OK: {passed}/{total}")
    
    if passed == total:
        print("\n  [SUCCESS] Todos los modulos de la arquitectura cognitiva funcionan correctamente.")
        return True
    else:
        print("\n  [WARNING] Algunos modulos tienen errores:")
        for name, result in results.items():
            if result != "OK":
                print(f"    - {name}: {result}")
        return False


def test_cognitive_system():
    """Test full cognitive system creation."""
    print("\n" + "=" * 60)
    print("ATLAS COGNITIVE SYSTEM - Full System Test")
    print("=" * 60)
    
    try:
        from modules.humanoid import create_cognitive_system
        
        print("\n  Creando sistema cognitivo completo...")
        system = create_cognitive_system()
        
        print(f"  - Medulla: {type(system['medulla']).__name__}")
        print(f"  - Cortex modules: {len(system['cortex'])}")
        print(f"  - Limbic modules: {len(system['limbic'])}")
        print(f"  - Hippo: {type(system['hippo']).__name__}")
        print(f"  - Brainstem modules: {len(system['brainstem'])}")
        print(f"  - Basal modules: {len(system['basal'])}")
        print(f"  - Motor modules: {len(system['motor'])}")
        print(f"  - Learning: {type(system['learning']).__name__}")
        
        print("\n  [SUCCESS] Sistema cognitivo creado correctamente.")
        return True
        
    except Exception as e:
        print(f"\n  [FAIL] Error creando sistema: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print()
    
    imports_ok = test_imports()
    system_ok = test_cognitive_system()
    
    print("\n" + "=" * 60)
    if imports_ok and system_ok:
        print("ATLAS COGNITIVE SYSTEM: READY")
    else:
        print("ATLAS COGNITIVE SYSTEM: NEEDS ATTENTION")
    print("=" * 60 + "\n")
    
    sys.exit(0 if (imports_ok and system_ok) else 1)
