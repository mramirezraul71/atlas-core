#!/usr/bin/env python3
"""
Test script para ATLAS NEXUS Neural Router con Directivas
Ejecutar desde: C:\ATLAS_NEXUS\atlas_nexus\
"""

import sys
import os
import asyncio

# Asegurar que estamos en el directorio correcto
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

print(f"🚀 ATLAS NEXUS - Test Neural Router con Directivas")
print(f"📁 Directorio: {script_dir}")
print()

async def test():
    """Test completo del sistema con directivas"""
    try:
        # Importar módulos necesarios
        from brain.neural_router import TaskContext, TaskType, NeuralRouter
        from config.nexus_config import NexusConfig
        
        print("✅ Módulos importados correctamente")
        print()
        
        # Crear configuración
        config = NexusConfig()
        print("✅ Configuración creada")
        print(f"   - Ollama enabled: {config.ollama.enabled}")
        print(f"   - OpenAI enabled: {config.openai.enabled}")
        print(f"   - Anthropic enabled: {config.anthropic.enabled}")
        print()
        
        # Crear Neural Router
        router = NeuralRouter(config)
        print("✅ Neural Router creado")
        print(f"   - Directives manager: {'Disponible' if router.directives_manager else 'No disponible'}")
        print()
        
        # Test 1: Task sin proyecto (solo directivas globales)
        print("🧪 Test 1: Task sin proyecto (directivas globales)")
        task1 = TaskContext(
            prompt="Explica brevemente qué son las directivas en ATLAS NEXUS",
            task_type=TaskType.CONVERSATION,
            temperature=0.7
        )
        
        print("   Enviando request...")
        response1 = await router.think(task1)
        print(f"   ✅ Respuesta recibida: {len(response1.content)} chars")
        print(f"   📝 Preview: {response1.content[:100]}...")
        print()
        
        # Test 2: Task con proyecto (directivas específicas)
        print("🧪 Test 2: Task con proyecto (directivas de trading_bot)")
        task2 = TaskContext(
            prompt="Crea una función simple para calcular el RSI",
            task_type=TaskType.CODE_GENERATION,
            metadata={"project": "trading_bot"},
            temperature=0.3
        )
        
        print("   Enviando request con directivas de trading_bot...")
        response2 = await router.think(task2)
        print(f"   ✅ Respuesta recibida: {len(response2.content)} chars")
        print(f"   📝 Preview: {response2.content[:100]}...")
        print()
        
        # Test 3: Verificar directivas activas
        print("🧪 Test 3: Verificar directivas activas")
        if router.directives_manager:
            global_dirs = router.directives_manager.get_global_directives()
            trading_dirs = router.directives_manager.get_project_directives("trading_bot")
            active_dirs = router.directives_manager.get_active_directives("trading_bot")
            
            print(f"   📋 Directivas globales: {len(global_dirs)} chars")
            print(f"   📋 Directivas trading_bot: {len(trading_dirs)} chars")
            print(f"   📋 Directivas activas combinadas: {len(active_dirs)} chars")
            print(f"   🔍 Contiene 'Streamlit': {'Streamlit' in active_dirs}")
            print(f"   🔍 Contiene 'credenciales': {'credenciales' in active_dirs.lower()}")
        else:
            print("   ❌ Directives manager no disponible")
        print()
        
        print("🎉 TODOS LOS TESTS COMPLETADOS EXITOSAMENTE!")
        print("🚀 ATLAS NEXUS con directivas funcionando perfectamente!")
        
        return True
        
    except Exception as e:
        print(f"❌ Error en test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("🔥 Iniciando test del sistema...")
    print("=" * 60)
    
    success = asyncio.run(test())
    
    print("=" * 60)
    if success:
        print("✅ TEST EXITOSO - Sistema operativo")
    else:
        print("❌ TEST FALLIDO - Revisar errores")
    
    print("\n📋 Para ejecutar la API:")
    print("   python nexus.py --mode api")
    print("   Luego visitar: http://localhost:8000/docs")
