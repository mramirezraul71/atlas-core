"""
ATLAS NEXUS - Demo & Testing Script
Demonstrates key capabilities
"""

import asyncio
import sys
from pathlib import Path

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent))

from nexus import AtlasNexus
from config.nexus_config import NexusConfig, Environment

async def demo_basic_thinking():
    """Demo 1: Basic AI thinking"""
    print("\n" + "="*60)
    print("DEMO 1: Basic AI Thinking")
    print("="*60)
    
    nexus = AtlasNexus()
    
    prompts = [
        "Explain what ATLAS NEXUS is in one sentence",
        "What is 15 * 37?",
        "Give me 3 tips for productivity"
    ]
    
    for prompt in prompts:
        print(f"\n📝 Prompt: {prompt}")
        response = await nexus.think(prompt)
        print(f"🤖 Response: {response[:200]}...")

async def demo_code_generation():
    """Demo 2: Code generation (uses DeepSeek Coder)"""
    print("\n" + "="*60)
    print("DEMO 2: Code Generation")
    print("="*60)
    
    nexus = AtlasNexus()
    
    prompt = "Write a Python function to calculate fibonacci numbers"
    print(f"\n📝 Request: {prompt}")
    
    response = await nexus.think(prompt, task_type="code_generation")
    print(f"\n🤖 Generated Code:\n{response}")

async def demo_autonomous_goal():
    """Demo 3: Autonomous goal execution"""
    print("\n" + "="*60)
    print("DEMO 3: Autonomous Goal Execution")
    print("="*60)
    
    nexus = AtlasNexus()
    
    goal = "Create a simple to-do list system design"
    print(f"\n🎯 Goal: {goal}")
    print("⏳ Planning and executing...")
    
    plan = await nexus.achieve_goal(goal)
    
    print(f"\n✅ Completed!")
    print(f"Plan ID: {plan.id}")
    print(f"Status: {plan.status.value}")
    print(f"Total Steps: {len(plan.steps)}")
    
    print("\n📋 Steps Executed:")
    for i, step in enumerate(plan.steps, 1):
        print(f"  {i}. {step.description} - {step.status.value}")
        if step.result:
            result_preview = str(step.result)[:100]
            print(f"     Result: {result_preview}...")

async def demo_tools():
    """Demo 4: Tool execution"""
    print("\n" + "="*60)
    print("DEMO 4: Tool Execution")
    print("="*60)
    
    nexus = AtlasNexus()
    
    # Demo web search tool
    print("\n🔍 Testing web_search tool...")
    result = await nexus.tools_registry.execute_tool(
        name="web_search",
        parameters={"query": "artificial intelligence", "max_results": 3},
        context={}
    )
    print(f"Results: {len(result.get('results', []))} items found")
    
    # Demo file write tool  
    print("\n📝 Testing file_write tool...")
    test_content = "This is a test file created by ATLAS NEXUS\nTimestamp: {}"
    from datetime import datetime
    result = await nexus.tools_registry.execute_tool(
        name="file_write",
        parameters={
            "path": "memory/test_demo.txt",
            "content": test_content.format(datetime.now())
        },
        context={}
    )
    print(f"File written: {result.get('path')}")
    
    # List available tools
    print("\n🛠️  Available Tools:")
    manifest = nexus.tools_registry.get_tools_manifest()
    print(f"Total tools: {manifest['total_tools']}")
    print("\nBy category:")
    for cat, count in manifest['categories'].items():
        print(f"  - {cat}: {count} tools")

async def demo_system_status():
    """Demo 5: System status"""
    print("\n" + "="*60)
    print("DEMO 5: System Status")
    print("="*60)
    
    nexus = AtlasNexus()
    status = nexus.get_status()
    
    print(f"\n📊 System Status:")
    print(f"  Status: {status['status']}")
    print(f"  Tools Available: {status['tools_available']}")
    print(f"  Active Plans: {status['active_plans']}")
    
    print(f"\n🧠 AI Providers:")
    providers = status['config']['llm_providers']
    for provider, enabled in providers.items():
        emoji = "✅" if enabled else "❌"
        print(f"  {emoji} {provider}")

async def run_all_demos():
    """Run all demos"""
    print("\n")
    print("╔" + "="*58 + "╗")
    print("║" + " "*15 + "ATLAS NEXUS DEMO" + " "*27 + "║")
    print("║" + " "*10 + "Professional AI System Testing" + " "*17 + "║")
    print("╚" + "="*58 + "╝")
    
    try:
        await demo_system_status()
        await demo_basic_thinking()
        # await demo_code_generation()  # Uncomment if DeepSeek is available
        await demo_autonomous_goal()
        await demo_tools()
        
        print("\n" + "="*60)
        print("✅ All demos completed successfully!")
        print("="*60)
        print("\n💡 Next steps:")
        print("  1. Start API server: python nexus.py --mode api")
        print("  2. Try interactive mode: python nexus.py --mode interactive")
        print("  3. Check API docs: http://localhost:8000/docs")
        print("  4. Connect from mobile: Use REST API")
        
    except Exception as e:
        print(f"\n❌ Error during demo: {e}")
        print("\n🔍 Troubleshooting:")
        print("  • Make sure Ollama is running: ollama list")
        print("  • Check config/.env is configured")
        print("  • Install dependencies: pip install -r requirements.txt")

async def quick_test():
    """Quick functionality test"""
    print("🚀 Quick Test - Testing core functionality...\n")
    
    try:
        # Test 1: Import check
        print("✓ Imports successful")
        
        # Test 2: Config
        config = NexusConfig()
        print(f"✓ Configuration loaded (Root: {config.paths.root})")
        
        # Test 3: Initialize
        nexus = AtlasNexus(config)
        print("✓ ATLAS NEXUS initialized")
        
        # Test 4: Tools
        tools_count = len(nexus.tools_registry.tools)
        print(f"✓ Tools registry loaded ({tools_count} tools)")
        
        # Test 5: Think
        response = await nexus.think("Say 'Test successful'")
        print(f"✓ Neural router working")
        print(f"  Response preview: {response[:50]}...")
        
        print("\n🎉 All tests passed! System is operational.")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--quick":
        # Quick test mode
        asyncio.run(quick_test())
    else:
        # Full demo mode
        asyncio.run(run_all_demos())
