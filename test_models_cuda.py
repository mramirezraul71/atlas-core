#!/usr/bin/env python3
"""Test de modelos IA con CUDA."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import torch

print("=== VERIFICACION MODELOS IA ===")
print(f"PyTorch: {torch.__version__}")
print(f"CUDA: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"VRAM: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
print()

results = {}

# Test MiDaS
print("1. MiDaS (Profundidad)...")
try:
    model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
    model.cuda().eval()
    print("   OK - MiDaS cargado en GPU")
    results["midas"] = True
except Exception as e:
    print(f"   ERROR: {e}")
    results["midas"] = False

# Test CLIP
print("2. CLIP (Embeddings)...")
try:
    import clip
    model, preprocess = clip.load("ViT-B/32", device="cuda")
    print("   OK - CLIP cargado en GPU")
    results["clip"] = True
except ImportError:
    try:
        from transformers import CLIPModel
        model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        model.cuda().eval()
        print("   OK - CLIP (transformers) cargado en GPU")
        results["clip"] = True
    except Exception as e:
        print(f"   ERROR: {e}")
        results["clip"] = False
except Exception as e:
    print(f"   ERROR: {e}")
    results["clip"] = False

# Test Whisper
print("3. Whisper (STT)...")
try:
    import whisper
    model = whisper.load_model("base", device="cuda")
    print("   OK - Whisper cargado en GPU")
    results["whisper"] = True
except Exception as e:
    print(f"   ERROR: {e}")
    results["whisper"] = False

# Test YOLO
print("4. YOLO (Deteccion)...")
try:
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")
    model.to("cuda")
    print("   OK - YOLOv8n cargado en GPU")
    results["yolo"] = True
except Exception as e:
    print(f"   ERROR: {e}")
    results["yolo"] = False

print()
print("=== RESUMEN ===")
ok_count = sum(1 for v in results.values() if v)
total = len(results)
print(f"Modelos OK: {ok_count}/{total}")

for name, ok in results.items():
    status = "OK" if ok else "FALLO"
    print(f"  {name}: {status}")

print()
if ok_count == total:
    print("TODOS LOS MODELOS FUNCIONANDO CON CUDA")
else:
    print("Algunos modelos tienen problemas")
