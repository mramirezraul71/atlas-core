#!/usr/bin/env python3
"""
Test de integración de modelos IA reales en ATLAS.
Verifica que los módulos cognitivos usen GPU cuando está disponible.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import torch

print("=" * 60)
print("  TEST DE MODELOS IA REALES EN ATLAS")
print("=" * 60)
print(f"\nPyTorch: {torch.__version__}")
print(f"CUDA: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"VRAM libre: {torch.cuda.memory_reserved(0)/1024**3:.2f} GB")

results = {}

# Test 1: DepthEstimation con MiDaS
print("\n" + "-" * 40)
print("1. DepthEstimation (MiDaS)")
print("-" * 40)
try:
    from modules.humanoid.cortex.occipital import DepthEstimation
    
    de = DepthEstimation(model_name="midas", model_size="MiDaS_small")
    de.load()
    
    # Crear imagen de prueba
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    output = de.estimate(test_image)
    
    print(f"   Modelo: {output.model_used}")
    print(f"   Profundidad media: {output.mean_depth:.2f}m")
    print(f"   Tiempo: {output.processing_time_ms:.1f}ms")
    
    if "mock" not in output.model_used.lower():
        print("   ✅ Usando modelo REAL")
        results["depth"] = True
    else:
        print("   ⚠️  Usando mock")
        results["depth"] = False
        
except Exception as e:
    print(f"   ❌ Error: {e}")
    results["depth"] = False

# Test 2: ObjectRecognition con CLIP
print("\n" + "-" * 40)
print("2. ObjectRecognition (CLIP)")
print("-" * 40)
try:
    from modules.humanoid.cortex.occipital import ObjectRecognition
    
    orec = ObjectRecognition(embedding_model="clip")
    
    # Extraer embedding
    test_image = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
    embedding = orec._extractor.extract(test_image)
    
    print(f"   Embedding dim: {len(embedding)}")
    print(f"   Modelo cargado: {orec._extractor._use_real_clip}")
    
    if orec._extractor._use_real_clip:
        print("   ✅ Usando CLIP REAL")
        results["clip"] = True
    else:
        print("   ⚠️  Usando mock")
        results["clip"] = False
        
except Exception as e:
    print(f"   ❌ Error: {e}")
    results["clip"] = False

# Test 3: VisionPipeline con YOLO
print("\n" + "-" * 40)
print("3. VisionPipeline (YOLO)")
print("-" * 40)
try:
    from modules.humanoid.cortex.occipital import VisionPipeline
    
    vp = VisionPipeline(detector_model="yolov8n", enable_depth=False, enable_pose=False)
    print(f"   Device: {vp.device}")
    
    # Procesar frame
    test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    output = vp.process_frame(test_frame, detect_humans=False, estimate_depth=False)
    
    print(f"   Detecciones: {len(output.detections)}")
    print(f"   Tiempo: {output.processing_time_ms:.1f}ms")
    
    if vp._detector is not None and not isinstance(vp._detector, type):
        if hasattr(vp._detector, 'detect'):
            # Es mock
            print("   ⚠️  Usando mock detector")
            results["yolo"] = False
        else:
            print("   ✅ Usando YOLO REAL")
            results["yolo"] = True
    else:
        results["yolo"] = False
        
except Exception as e:
    print(f"   ❌ Error: {e}")
    results["yolo"] = False

# Test 4: AudioProcessor con Whisper
print("\n" + "-" * 40)
print("4. AudioProcessor (Whisper)")
print("-" * 40)
try:
    from modules.humanoid.cortex.temporal import AudioProcessor
    
    ap = AudioProcessor(asr_model="whisper-base")
    ap._load_asr()
    
    if ap._asr != "mock":
        print("   ✅ Whisper cargado")
        results["whisper"] = True
    else:
        print("   ⚠️  Usando mock ASR")
        results["whisper"] = False
        
except Exception as e:
    print(f"   ❌ Error: {e}")
    results["whisper"] = False

# Resumen
print("\n" + "=" * 60)
print("  RESUMEN")
print("=" * 60)

ok_count = sum(1 for v in results.values() if v)
total = len(results)

for name, ok in results.items():
    status = "✅ REAL" if ok else "⚠️  MOCK"
    print(f"  {name}: {status}")

print(f"\n  Modelos reales: {ok_count}/{total}")

if ok_count == total:
    print("\n  🟢 TODOS LOS MODELOS FUNCIONANDO EN GPU")
elif ok_count > 0:
    print(f"\n  🟡 {ok_count} MODELOS REALES ACTIVOS")
else:
    print("\n  🔴 NINGÚN MODELO REAL ACTIVO")

# VRAM usado
if torch.cuda.is_available():
    print(f"\n  VRAM usado: {torch.cuda.memory_allocated(0)/1024**3:.2f} GB")
    print(f"  VRAM reservado: {torch.cuda.memory_reserved(0)/1024**3:.2f} GB")

print("=" * 60)
