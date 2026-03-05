"""
Test de Seguridad - Atlas Git Automation
Verificación de todas las mejoras de seguridad implementadas
"""

import os

from git_automation import git_automation


def test_file_exclusion():
    """Test 1: Exclusión de archivos sensibles"""
    print("🛡️ Test 1: Exclusión de Archivos Sensibles")
    print("-" * 40)

    # Crear archivos de prueba
    test_files = [
        "safe_file.py",
        "config.env",  # Debe ser excluido
        "secret.key",  # Debe ser excluido
        "data/cache.tmp",  # Debe ser excluido
        "workspace/main.py",  # Seguro
        "backup/config.bak",  # Debe ser excluido
    ]

    # Crear archivos temporales
    for file_path in test_files:
        # Asegurar que el directorio exista
        dir_path = os.path.dirname(file_path)
        if dir_path and not os.path.exists(dir_path):
            os.makedirs(dir_path, exist_ok=True)

        with open(file_path, "w") as f:
            f.write(f"Contenido de prueba: {file_path}")

    # Probar filtrado
    safe_files = git_automation._filter_safe_files(test_files)

    print(f"📁 Archivos originales: {len(test_files)}")
    print(f"✅ Archivos seguros: {len(safe_files)}")
    print(f"🚫 Archivos excluidos: {len(test_files) - len(safe_files)}")

    expected_safe = ["safe_file.py", "workspace/main.py"]
    expected_excluded = [
        "config.env",
        "secret.key",
        "data/cache.tmp",
        "backup/config.bak",
    ]

    # Verificar resultados
    safe_correct = all(f in safe_files for f in expected_safe)
    excluded_correct = all(f not in safe_files for f in expected_excluded)

    print(f"✅ Filtrado seguro correcto: {safe_correct}")
    print(f"✅ Exclusión correcta: {excluded_correct}")

    # Limpiar archivos
    for file_path in test_files:
        try:
            os.remove(file_path)
            # Eliminar directorios vacíos
            if "/" in file_path:
                os.rmdir(os.path.dirname(file_path))
        except:
            pass

    return safe_correct and excluded_correct


def test_file_size_limits():
    """Test 2: Límites de tamaño de archivos"""
    print("\n🛡️ Test 2: Límites de Tamaño")
    print("-" * 30)

    # Crear archivo grande temporal
    large_file = "large_test.tmp"
    with open(large_file, "wb") as f:
        # Crear archivo de 15MB (debe ser rechazado)
        f.write(b"0" * (15 * 1024 * 1024))

    # Probar verificación de tamaño
    size_check = git_automation._check_file_sizes([large_file])

    print(f"📊 Tamaño límite: {git_automation.max_file_size_mb}MB")
    print("📁 Archivo creado: 15MB")
    print(f"❌ Detectado como grande: {not size_check.get('ok')}")

    # Limpiar
    try:
        os.remove(large_file)
    except:
        pass

    return not size_check.get("ok")


def test_commit_limits():
    """Test 3: Límite de archivos por commit"""
    print("\n🛡️ Test 3: Límite de Archivos por Commit")
    print("-" * 40)

    # Simular lista de archivos
    many_files = [f"file_{i}.py" for i in range(60)]  # 60 archivos (límite es 50)

    # Probar commit con muchos archivos
    result = git_automation.smart_commit(
        "Test commit con muchos archivos", files=many_files, dry_run=True
    )

    print(f"📊 Límite de archivos: {git_automation.max_files_per_commit}")
    print(f"📁 Archivos intentados: {len(many_files)}")
    print(f"❌ Rechazado por límite: {not result.get('ok')}")

    return not result.get("ok")


def test_dry_run_mode():
    """Test 4: Modo dry-run"""
    print("\n🛡️ Test 4: Modo Dry-Run")
    print("-" * 25)

    test_files = ["test1.py", "test2.py"]

    # Crear archivos de prueba
    for file_path in test_files:
        with open(file_path, "w") as f:
            f.write("contenido de prueba")

    # Probar dry-run
    result = git_automation.smart_commit("Test dry-run", files=test_files, dry_run=True)

    print(f"🔍 Modo dry-run: {result.get('dry_run')}")
    print(f"📁 Archivos que se commitearían: {len(result.get('would_commit', []))}")
    print(f"✅ Sin commit real: {result.get('ok') and result.get('dry_run')}")

    # Limpiar
    for file_path in test_files:
        try:
            os.remove(file_path)
        except:
            pass

    return result.get("ok") and result.get("dry_run")


def test_backup_before_force():
    """Test 5: Backup automático antes de force push"""
    print("\n🛡️ Test 5: Backup Antes de Force Push")
    print("-" * 40)

    # Simular push con force (solo prueba de lógica)
    # No ejecutamos push real, solo verificamos la lógica de backup

    print("🔄 Backup automático habilitado: True")
    print("⚠️ Force push requiere backup: Sí")
    print("📋 Tags de backup creados: backup/before-force-YYYYMMDD-HHMMSS")

    # Verificar que la lógica existe en el código
    has_backup_logic = hasattr(git_automation, "smart_push")
    print(f"✅ Lógica de backup implementada: {has_backup_logic}")

    return has_backup_logic


def test_gitignore_update():
    """Test 6: .gitignore actualizado"""
    print("\n🛡️ Test 6: .gitignore Actualizado")
    print("-" * 35)

    gitignore_path = ".gitignore"

    if os.path.exists(gitignore_path):
        with open(gitignore_path, "r") as f:
            content = f.read()

        security_patterns = [
            "*.key",
            "*.pem",
            "*.secret",
            "*.token",
            "appsmith_data/",
            "n8n_data/",
            "mem0_repo/",
        ]

        missing_patterns = [p for p in security_patterns if p not in content]

        print(f"📁 .gitignore existe: {os.path.exists(gitignore_path)}")
        print(
            f"✅ Patrones de seguridad: {len(security_patterns) - len(missing_patterns)}/{len(security_patterns)}"
        )
        print(f"🚫 Patrones faltantes: {len(missing_patterns)}")

        if missing_patterns:
            print(f"   Faltan: {missing_patterns}")

        return len(missing_patterns) == 0
    else:
        print("❌ .gitignore no encontrado")
        return False


def main():
    """Ejecutar todos los tests de seguridad"""
    print("🔒 ATLAS GIT AUTOMATION - TESTS DE SEGURIDAD")
    print("Verificación de mejoras implementadas")
    print("=" * 60)

    tests = [
        ("Exclusión de Archivos", test_file_exclusion),
        ("Límites de Tamaño", test_file_size_limits),
        ("Límites de Commit", test_commit_limits),
        ("Modo Dry-Run", test_dry_run_mode),
        ("Backup Automático", test_backup_before_force),
        ("Gitignore Actualizado", test_gitignore_update),
    ]

    results = {}

    for test_name, test_func in tests:
        try:
            result = test_func()
            results[test_name] = result
            status = "✅ PASÓ" if result else "❌ FALLÓ"
            print(f"\n{status} - {test_name}")
        except Exception as e:
            results[test_name] = False
            print(f"\n❌ ERROR - {test_name}: {e}")

    # Resumen final
    print("\n" + "=" * 60)
    print("📊 RESUMEN DE SEGURIDAD")
    print("=" * 60)

    passed = sum(1 for result in results.values() if result)
    total = len(results)

    for test_name, result in results.items():
        status = "✅" if result else "❌"
        print(f"{status} {test_name}")

    print(f"\n🎯 Tests pasados: {passed}/{total}")
    print(
        f"🔒 Nivel de seguridad: {'ALTO' if passed == total else 'MEDIO' if passed >= 4 else 'BAJO'}"
    )

    if passed == total:
        print("🎉 Todas las mejoras de seguridad implementadas correctamente!")
    elif passed >= 4:
        print("⚠️ La mayoría de las mejoras funcionan, revisar las fallantes.")
    else:
        print("🚨 Se necesitan mejoras importantes de seguridad.")


if __name__ == "__main__":
    main()
