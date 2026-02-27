#!/usr/bin/env python3
"""
Ayuda para conexión de cámara - Diagnóstico y configuración.
Ejecutar desde backend: python scripts/camera_connect_helper.py
"""
import sys
import os
import json
import subprocess

# Añadir backend al path
BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BACKEND)
os.chdir(BACKEND)


def run():
    print("=" * 60)
    print("ATLAS Robot - Asistente de conexión de cámara")
    print("=" * 60)

    # 1. Verificar OpenCV
    try:
        import cv2
        print("\n[OK] OpenCV instalado:", cv2.__version__)
    except ImportError:
        print("\n[ERROR] OpenCV no instalado. Ejecuta: pip install opencv-python")
        return 1

    # 2. Detectar cámaras vía PnP (Windows)
    pnp_devices = []
    if sys.platform == "win32":
        try:
            ps = subprocess.run(
                [
                    "powershell", "-NoProfile", "-Command",
                    "Get-PnpDevice -Class Camera -EA SilentlyContinue | "
                    "Select-Object Status,FriendlyName | ConvertTo-Json -Compress; "
                    "Get-PnpDevice -Class Image -EA SilentlyContinue | "
                    "Select-Object Status,FriendlyName | ConvertTo-Json -Compress",
                ],
                capture_output=True, text=True, timeout=5,
            )
            if ps.returncode == 0 and ps.stdout:
                try:
                    data = json.loads("[" + ps.stdout.replace("}\n{", "},{") + "]")
                    for d in (data if isinstance(data, list) else [data]):
                        name = (d.get("FriendlyName") or "").strip()
                        if name and d.get("Status") == "OK":
                            pnp_devices.append({"name": name, "status": "OK"})
                except json.JSONDecodeError:
                    pass
        except Exception as e:
            print(f"\n[AVISO] No se pudo leer PnP: {e}")

    if pnp_devices:
        print(f"\n[OK] Dispositivos PnP detectados ({len(pnp_devices)}):")
        for d in pnp_devices:
            print(f"     - {d['name']}")
    else:
        print("\n[AVISO] No se encontraron dispositivos PnP Camera/Image.")

    # 3. Escanear índices OpenCV
    backend = cv2.CAP_MSMF if hasattr(cv2, "CAP_MSMF") and sys.platform == "win32" else cv2.CAP_ANY
    backend_name = "MSMF (Windows)" if backend == getattr(cv2, "CAP_MSMF", -1) else "ANY"
    print(f"\n[INFO] Escaneando índices OpenCV (backend: {backend_name})...")

    available = []
    for i in range(8):
        cap = cv2.VideoCapture(i, backend)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            cap.release()
            available.append({"index": i, "resolution": [w or 640, h or 480]})
            print(f"     [OK] Índice {i}: {w or 640}x{h or 480}")

    if not available:
        print("     [ERROR] No se encontró ninguna cámara con OpenCV.")
        print("\nPasos sugeridos:")
        print("  1. Verifica que la cámara esté conectada por USB.")
        print("  2. Comprueba en Administrador de dispositivos que no haya conflictos.")
        print("  3. Si es Insta360 Link, prueba conectar en otro puerto USB 3.0.")
        print("  4. Cierra otras apps que usen la cámara (Zoom, Teams, etc.).")
        return 1

    # 4. Probar conexión real (lectura de frame)
    print("\n[INFO] Probando lectura de frames...")
    idx = available[0]["index"]
    cap = cv2.VideoCapture(idx, backend)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            print(f"     [OK] Índice {idx}: lectura correcta, frame {frame.shape[1]}x{frame.shape[0]}")
        else:
            print(f"     [AVISO] Índice {idx}: abre pero no devuelve frames.")
    else:
        print(f"     [ERROR] No se pudo abrir índice {idx}.")
        return 1

    # 5. Configurar cámara activa
    try:
        from vision.cameras.detector import detect_cameras, save_active_config
        detected = detect_cameras()
        print(f"\n[OK] Detección completa: {len(detected)} cámara(s)")
        for c in detected:
            print(f"     - Índice {c.get('index')}: {c.get('model', 'Webcam')}")

        if detected:
            config = {
                "index": detected[0]["index"],
                "model": detected[0].get("model", "Webcam estándar"),
                "resolution": detected[0].get("resolution", [640, 480]),
                "capabilities": detected[0].get("capabilities", ["video"]),
            }
            if save_active_config(config):
                print(f"\n[OK] Config guardada: active_camera.json (índice {config['index']})")
            else:
                print("\n[AVISO] No se pudo guardar config.")
    except Exception as e:
        print(f"\n[AVISO] Detección del módulo: {e}")

    print("\n" + "=" * 60)
    print("Siguiente paso: reinicia el backend Robot (puerto 8002) para usar la cámara.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(run())
