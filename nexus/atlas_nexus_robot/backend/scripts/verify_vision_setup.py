"""
Verificación de setup para Vision Ubiq V2.
Ejecutar ANTES de comenzar implementación.

Uso:
    python scripts/verify_vision_setup.py
"""
import sys
from pathlib import Path

# Agregar backend al path
sys.path.insert(0, str(Path(__file__).parent.parent))


def check_python_version():
    """Verifica Python 3.10+"""
    version = sys.version_info
    if version.major < 3 or (version.major == 3 and version.minor < 10):
        print("❌ Python 3.10+ requerido")
        return False
    print(f"✅ Python {version.major}.{version.minor}.{version.micro}")
    return True


def check_opencv():
    """Verifica OpenCV instalado y funcional"""
    try:
        import cv2
        version = cv2.__version__
        # Test básico de captura
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.release()
            print(f"✅ OpenCV {version} (cámara funcional)")
            return True
        else:
            print(f"⚠️  OpenCV {version} (sin cámara detectada)")
            return True  # No es crítico para desarrollo
    except ImportError:
        print("❌ OpenCV no instalado")
        return False


def check_ffmpeg():
    """Verifica FFmpeg en PATH"""
    import subprocess
    try:
        result = subprocess.run(
            ["ffmpeg", "-version"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode == 0:
            version_line = result.stdout.split('\n')[0]
            print(f"✅ FFmpeg instalado: {version_line}")
            return True
    except FileNotFoundError:
        print("❌ FFmpeg no encontrado en PATH")
        return False
    except Exception as e:
        print(f"⚠️  FFmpeg error: {e}")
        return False


def check_dependencies():
    """Verifica dependencias principales"""
    required = [
        ("fastapi", "FastAPI"),
        ("uvicorn", "Uvicorn"),
        ("numpy", "NumPy"),
        ("PIL", "Pillow"),
    ]
    
    all_ok = True
    for module_name, display_name in required:
        try:
            __import__(module_name)
            print(f"✅ {display_name}")
        except ImportError:
            print(f"❌ {display_name} no instalado")
            all_ok = False
    
    return all_ok


def check_optional_dependencies():
    """Verifica dependencias opcionales (FASE 3+)"""
    optional = [
        ("zeroconf", "Zeroconf (mDNS)", "FASE 3"),
        ("miniupnpc", "MiniUPnPC (UPnP)", "FASE 3 (opcional)"),
    ]
    
    print("\n--- Dependencias Opcionales ---")
    for module_name, display_name, phase in optional:
        try:
            __import__(module_name)
            print(f"✅ {display_name} - {phase}")
        except ImportError:
            print(f"⏭️  {display_name} - {phase} (instalar cuando necesario)")


def check_git_status():
    """Verifica estado de git"""
    import subprocess
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if result.returncode == 0:
            changes = result.stdout.strip()
            if not changes:
                print("✅ Git: sin cambios pendientes")
            else:
                print("⚠️  Git: hay cambios sin commitear")
                print("   Recomendación: hacer commit antes de empezar")
        return True
    except Exception:
        print("⚠️  Git: no disponible")
        return True  # No crítico


def check_backup_files():
    """Verifica si existen backups de archivos críticos"""
    critical_files = [
        "api/vision_routes.py",
        "vision/cameras/factory.py",
    ]
    
    backend_dir = Path(__file__).parent.parent
    print("\n--- Archivos Críticos ---")
    
    for file_path in critical_files:
        full_path = backend_dir / file_path
        backup_path = Path(str(full_path) + ".backup")
        
        if full_path.exists():
            if backup_path.exists():
                print(f"✅ {file_path} (backup existe)")
            else:
                print(f"⚠️  {file_path} (crear backup recomendado)")
        else:
            print(f"❌ {file_path} (no encontrado)")


def check_ports():
    """Verifica puertos disponibles"""
    import socket
    
    ports = [
        (8000, "ATLAS Adapter"),
        (8002, "ATLAS Nexus Robot"),
    ]
    
    print("\n--- Puertos ---")
    for port, service in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = sock.connect_ex(('127.0.0.1', port))
        sock.close()
        
        if result == 0:
            print(f"✅ Puerto {port} - {service} (en uso - OK)")
        else:
            print(f"⚠️  Puerto {port} - {service} (disponible - iniciar servicio)")


def main():
    print("=" * 60)
    print("  VISION UBIQ V2 - VERIFICACIÓN DE SETUP")
    print("=" * 60)
    print()
    
    checks = [
        ("Python version", check_python_version),
        ("OpenCV", check_opencv),
        ("FFmpeg", check_ffmpeg),
        ("Dependencias principales", check_dependencies),
        ("Git status", check_git_status),
    ]
    
    results = []
    for name, check_func in checks:
        try:
            result = check_func()
            results.append(result)
        except Exception as e:
            print(f"❌ {name}: Error - {e}")
            results.append(False)
    
    # Checks informativos (no afectan resultado)
    check_optional_dependencies()
    check_backup_files()
    check_ports()
    
    print("\n" + "=" * 60)
    if all(results):
        print("✅ SETUP COMPLETO - Listo para implementar")
        print("\nPróximos pasos:")
        print("1. Leer docs/VISION_UBIQ_V2_ARQUITECTURA.md")
        print("2. Leer docs/VISION_UBIQ_V2_IMPLEMENTACION.md")
        print("3. Comenzar con FASE 1 (Optimización)")
        return 0
    else:
        print("❌ SETUP INCOMPLETO - Resolver errores antes de continuar")
        return 1


if __name__ == "__main__":
    sys.exit(main())
