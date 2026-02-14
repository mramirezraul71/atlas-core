"""
ATLAS NEXUS - Main Engine
Central orchestrator that brings everything together
"""

import asyncio
import logging
import sys
from pathlib import Path
from typing import Optional

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from config.nexus_config import NexusConfig, Environment
from brain.neural_router import NeuralRouter
from brain.autonomous_engine import AutonomousEngine
from tools.tools_registry import ToolsRegistry

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('logs/nexus.log')
    ]
)

logger = logging.getLogger("atlas.nexus")

class AtlasNexus:
    """
    ATLAS NEXUS - Main System
    
    Professional AI System with:
    - Multi-LLM Intelligence (Ollama, DeepSeek, Claude, GPT)
    - Autonomous Planning and Execution
    - 50+ Professional Tools
    - REST API for Mobile Control
    - Real-time WebSocket Updates
    - Self-Recovery and Auto-Learning
    """
    
    def __init__(self, config: Optional[NexusConfig] = None):
        """Initialize ATLAS NEXUS"""
        logger.info("=" * 60)
        logger.info("🚀 ATLAS NEXUS - Initializing...")
        logger.info("=" * 60)
        
        # Configuration
        self.config = config or NexusConfig()
        logger.info(f"Environment: {self.config.env.value}")
        logger.info(f"Root Path: {self.config.paths.root}")
        
        # Initialize Neural Router (Multi-LLM Brain)
        logger.info("\n🧠 Initializing Neural Router...")
        self.router = NeuralRouter(self.config)
        
        # Initialize Tools Registry
        logger.info("\n🛠️  Initializing Tools Registry...")
        self.tools_registry = ToolsRegistry()
        
        # Initialize Autonomous Engine
        logger.info("\n🤖 Initializing Autonomous Engine...")
        self.autonomous_engine = AutonomousEngine(
            config=self.config,
            router=self.router,
            tool_registry=self.tools_registry
        )
        
        logger.info("\n" + "=" * 60)
        logger.info("✅ ATLAS NEXUS - Ready!")
        logger.info("=" * 60 + "\n")
    
    async def start_api_server(self):
        """Start the REST API server"""
        from api.rest_api import create_api
        import uvicorn
        
        logger.info(f"🌐 Starting API Server on {self.config.api.host}:{self.config.api.port}")
        
        app = create_api(self)
        
        config = uvicorn.Config(
            app,
            host=self.config.api.host,
            port=self.config.api.port,
            log_level="info"
        )
        
        server = uvicorn.Server(config)
        await server.serve()
    
    async def achieve_goal(self, goal: str, context: dict = None):
        """Achieve a goal autonomously"""
        return await self.autonomous_engine.achieve_goal(goal, context)
    
    async def think(self, prompt: str, task_type: str = "conversation"):
        """Think using the neural router"""
        from brain.neural_router import TaskType, TaskContext
        
        try:
            tt = TaskType[task_type.upper()]
        except:
            tt = TaskType.CONVERSATION
        
        task_ctx = TaskContext(prompt=prompt, task_type=tt)
        response = await self.router.think(task_ctx)
        return response.content
    
    def get_status(self) -> dict:
        """Get system status"""
        return {
            "status": "operational",
            "config": self.config.to_dict(),
            "tools_available": len(self.tools_registry.tools),
            "active_plans": len(self.autonomous_engine.active_plans)
        }
    
    async def run_interactive(self):
        """Run in interactive mode"""
        logger.info("🎮 Interactive Mode - Type 'exit' to quit\n")
        
        while True:
            try:
                user_input = input("You: ")
                
                if user_input.lower() in ['exit', 'quit', 'q']:
                    logger.info("Shutting down...")
                    break
                
                if user_input.lower() == 'status':
                    status = self.get_status()
                    print(f"\nStatus: {status}\n")
                    continue
                
                # Use autonomous engine for complex requests
                if any(word in user_input.lower() for word in ['create', 'build', 'make', 'generate', 'write']):
                    logger.info("🤖 Executing autonomously...")
                    plan = await self.achieve_goal(user_input)
                    print(f"\n✅ Completed: {plan.goal}")
                    print(f"Status: {plan.status.value}")
                    print(f"Steps: {len(plan.steps)}\n")
                else:
                    # Simple conversation
                    response = await self.think(user_input)
                    print(f"\nAtlas: {response}\n")
                
            except KeyboardInterrupt:
                logger.info("\nShutting down...")
                break
            except Exception as e:
                logger.error(f"Error: {e}")
                print(f"\nError: {e}\n")
    
    async def run(self, mode: str = "api"):
        """Run ATLAS NEXUS in specified mode"""
        if mode == "api":
            await self.start_api_server()
        elif mode == "interactive":
            await self.run_interactive()
        else:
            raise ValueError(f"Unknown mode: {mode}")

# =========================
# MAIN
# =========================

async def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description="ATLAS NEXUS - Professional AI System")
    parser.add_argument("--mode", choices=["api", "interactive"], default="api",
                       help="Run mode: api (server) or interactive (CLI)")
    parser.add_argument("--env", choices=["development", "production", "testing"], 
                       default="development", help="Environment")
    
    args = parser.parse_args()
    
    # Create configuration
    env = Environment[args.env.upper()]
    config = NexusConfig(env=env)
    
    # Create and run ATLAS NEXUS
    nexus = AtlasNexus(config)
    
    try:
        await nexus.run(mode=args.mode)
    except KeyboardInterrupt:
        logger.info("\n👋 ATLAS NEXUS - Shutting down gracefully...")

if __name__ == "__main__":
    asyncio.run(main())
