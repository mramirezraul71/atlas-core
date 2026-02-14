"""
ATLAS NEXUS - Main Entry Point
Professional AI Assistant with Multi-LLM, Autonomous Execution, and Full Integration
"""

import asyncio
import os
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from config.nexus_config import config
from brain.neural_router import NeuralRouter
from brain.autonomous_engine import AutonomousEngine
from tools.tools_manager import ToolsManager


class AtlasNexus:
    """
    ATLAS NEXUS - Next Generation AI Assistant
    
    Features:
    - Multi-LLM orchestration (Claude, DeepSeek, Ollama)
    - Autonomous task execution
    - 50+ professional tools
    - REST API + WebSocket
    - Telegram integration
    - Memory persistence
    - Auto-recovery
    """
    
    def __init__(self):
        self.config = config
        self.version = "2.0.0"
        
        # Initialize core components
        print("🚀 Initializing ATLAS NEXUS...")
        
        self.router = NeuralRouter(config)
        print("  ✅ Neural Router initialized")
        
        self.tools = ToolsManager(config)
        print(f"  ✅ Tools Manager initialized ({len(self.tools.tools)} tools)")
        
        self.engine = AutonomousEngine(self.router, self.tools, config)
        print("  ✅ Autonomous Engine initialized")
        
        # Telegram bot (if enabled)
        self.telegram_bot = None
        
        print(f"\n{'='*60}")
        print(f"  ATLAS NEXUS v{self.version} - READY")
        print(f"{'='*60}\n")
    
    async def execute(self, user_input: str, context: dict = None):
        """
        Execute user command
        
        This is the main entry point for all user interactions
        """
        return await self.engine.execute(user_input, context)
    
    async def interactive_mode(self):
        """
        Interactive CLI mode for testing and development
        """
        print("\n🤖 ATLAS NEXUS - Interactive Mode")
        print("Type 'exit' to quit, 'help' for commands\n")
        
        while True:
            try:
                user_input = input("You: ").strip()
                
                if not user_input:
                    continue
                
                if user_input.lower() in ['exit', 'quit', 'salir']:
                    print("\n👋 ATLAS NEXUS shutting down...")
                    break
                
                if user_input.lower() == 'help':
                    self._show_help()
                    continue
                
                if user_input.lower() == 'stats':
                    self._show_stats()
                    continue
                
                if user_input.lower() == 'tools':
                    self._show_tools()
                    continue
                
                # Execute command
                print("\n🤖 ATLAS NEXUS: Processing...\n")
                result = await self.execute(user_input)
                
                if result['success']:
                    print(f"\n✅ Task completed!\n")
                    
                    # Show summary
                    if 'result' in result and result['result']:
                        exec_result = result['result']
                        if 'summary' in exec_result:
                            print(f"📝 Summary:\n{exec_result['summary']}\n")
                        elif 'execution' in exec_result:
                            print(f"📋 Executed {len(exec_result['execution'])} steps\n")
                else:
                    print(f"\n❌ Task failed: {result.get('error', 'Unknown error')}\n")
                
            except KeyboardInterrupt:
                print("\n\n👋 ATLAS NEXUS shutting down...")
                break
            except Exception as e:
                print(f"\n❌ Error: {e}\n")
    
    def _show_help(self):
        """Show help information"""
        print("""
╔══════════════════════════════════════════════════════════════╗
║                    ATLAS NEXUS - HELP                        ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  COMMANDS:                                                   ║
║    help     - Show this help                                 ║
║    stats    - Show system statistics                         ║
║    tools    - List available tools                           ║
║    exit     - Quit ATLAS NEXUS                              ║
║                                                              ║
║  EXAMPLES:                                                   ║
║    "Create a Python script to analyze CSV data"              ║
║    "Search the web for latest AI news"                       ║
║    "Analyze the file at C:/data/report.txt"                  ║
║    "Execute a multi-step plan to..."                         ║
║                                                              ║
║  FEATURES:                                                   ║
║    🧠 Multi-LLM: Claude, DeepSeek, Ollama                    ║
║    🤖 Autonomous: Self-planning and execution                ║
║    🛠️ 50+ Tools: Web, files, code, data, and more          ║
║    💾 Memory: Persistent context and learning                ║
║    🔄 Recovery: Auto-retry and self-correction               ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    def _show_stats(self):
        """Show system statistics"""
        stats = self.router.get_stats()
        
        print(f"""
╔══════════════════════════════════════════════════════════════╗
║                  ATLAS NEXUS - STATISTICS                    ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  LLM USAGE:                                                  ║
║    Total calls: {stats['total_calls']:<42} ║
║                                                              ║
║  CALLS BY MODEL:                                             ║
""")
        
        for model, count in stats.get('calls_by_model', {}).items():
            print(f"║    {model:<20} {count:<36} ║")
        
        print(f"""║                                                              ║
║  AVERAGE LATENCY:                                            ║
""")
        
        for model, latency in stats.get('avg_latency', {}).items():
            print(f"║    {model:<20} {latency:.3f}s{' ':<32} ║")
        
        print(f"""║                                                              ║
║  TASKS:                                                      ║
║    Total: {len(self.engine.tasks):<50} ║
║    Completed: {len([t for t in self.engine.tasks.values() if t.status.value == 'completed']):<46} ║
║    Failed: {len([t for t in self.engine.tasks.values() if t.status.value == 'failed']):<49} ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    def _show_tools(self):
        """Show available tools"""
        tools_by_category = {}
        for tool in self.tools.tools.values():
            if tool.category not in tools_by_category:
                tools_by_category[tool.category] = []
            tools_by_category[tool.category].append(tool)
        
        print(f"""
╔══════════════════════════════════════════════════════════════╗
║                 ATLAS NEXUS - AVAILABLE TOOLS                ║
╠══════════════════════════════════════════════════════════════╣
""")
        
        for category, tools in sorted(tools_by_category.items()):
            print(f"║                                                              ║")
            print(f"║  📁 {category.upper():<55} ║")
            for tool in tools:
                name = tool.name[:25]
                desc = tool.description[:30]
                print(f"║    • {name:<25} {desc:<27} ║")
        
        print(f"""║                                                              ║
╚══════════════════════════════════════════════════════════════╝
""")


async def main():
    """Main entry point"""
    # Initialize ATLAS NEXUS
    atlas = AtlasNexus()
    
    # Check command line arguments
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        
        if mode == 'api':
            # Start REST API server
            from api.rest_api import NexusAPI
            api = NexusAPI(atlas.engine, atlas.router, atlas.tools, atlas.config)
            api.run()
        
        elif mode == 'telegram':
            # Start Telegram bot
            print("🤖 Starting Telegram Bot...")
            # Import and start telegram module from original project
            from modules import atlas_telegram
            atlas_telegram.main()
        
        elif mode == 'gui':
            # Start GUI (placeholder)
            print("🖥️ GUI mode not yet implemented")
            print("Use 'python atlas_nexus.py' for interactive mode")
        
        else:
            print(f"Unknown mode: {mode}")
            print("Available modes: api, telegram, gui")
    
    else:
        # Interactive CLI mode
        await atlas.interactive_mode()


if __name__ == "__main__":
    try:
        # Run main async function
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 ATLAS NEXUS terminated by user")
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
