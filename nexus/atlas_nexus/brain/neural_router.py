"""
ATLAS NEXUS - Neural Router
Intelligent Multi-LLM Orchestrator with Smart Selection and Fallback
"""

import asyncio
import hashlib
import json
import time
from typing import Dict, List, Optional, Any, Tuple
from enum import Enum
from dataclasses import dataclass
import logging
import sys
import os

# Add the directives directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'directives'))

logger = logging.getLogger("atlas.brain.router")

class TaskType(Enum):
    """Types of tasks that can be routed"""
    CODE_GENERATION = "code_generation"
    CODE_REVIEW = "code_review"
    CODE_DEBUG = "code_debug"
    CONVERSATION = "conversation"
    ANALYSIS = "analysis"
    REASONING = "reasoning"
    QUICK_TASK = "quick_task"
    CREATIVE = "creative"
    PLANNING = "planning"
    EMBEDDING = "embedding"
    TRANSLATION = "translation"

@dataclass
class TaskContext:
    """Context for a task to be routed"""
    prompt: str
    task_type: TaskType
    system_prompt: Optional[str] = None
    temperature: float = 0.7
    max_tokens: int = 4096
    requires_accuracy: bool = False
    requires_speed: bool = False
    requires_creativity: bool = False
    metadata: Dict[str, Any] = None

@dataclass
class ModelResponse:
    """Response from a model"""
    content: str
    model: str
    provider: str
    tokens_used: int
    latency: float
    cached: bool = False
    error: Optional[str] = None

class ResponseCache:
    """Simple in-memory response cache"""
    
    def __init__(self, ttl: int = 3600):
        self.cache: Dict[str, Tuple[ModelResponse, float]] = {}
        self.ttl = ttl
    
    def _generate_key(self, prompt: str, task_type: str, model: str) -> str:
        """Generate cache key"""
        data = f"{prompt}:{task_type}:{model}"
        return hashlib.md5(data.encode()).hexdigest()
    
    def get(self, prompt: str, task_type: str, model: str) -> Optional[ModelResponse]:
        """Get cached response if not expired"""
        key = self._generate_key(prompt, task_type, model)
        if key in self.cache:
            response, timestamp = self.cache[key]
            if time.time() - timestamp < self.ttl:
                response.cached = True
                return response
            else:
                del self.cache[key]
        return None
    
    def set(self, prompt: str, task_type: str, model: str, response: ModelResponse):
        """Cache response"""
        key = self._generate_key(prompt, task_type, model)
        self.cache[key] = (response, time.time())
    
    def clear(self):
        """Clear all cache"""
        self.cache.clear()

class OllamaClient:
    """Client for Ollama LLMs"""
    
    def __init__(self, host: str = "http://localhost:11434", models: Dict[str, str] = None):
        self.host = host
        self.models = models or {}
        
    async def generate(self, prompt: str, model: str, system: Optional[str] = None, 
                      temperature: float = 0.7, max_tokens: int = 4096) -> ModelResponse:
        """Generate response using Ollama"""
        try:
            import aiohttp
            
            start = time.time()
            
            payload = {
                "model": model,
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": temperature,
                    "num_predict": max_tokens
                }
            }
            
            if system:
                payload["system"] = system
            
            async with aiohttp.ClientSession() as session:
                async with session.post(f"{self.host}/api/generate", json=payload) as resp:
                    if resp.status == 200:
                        data = await resp.json()
                        latency = time.time() - start
                        
                        return ModelResponse(
                            content=data.get("response", ""),
                            model=model,
                            provider="ollama",
                            tokens_used=data.get("eval_count", 0),
                            latency=latency
                        )
                    else:
                        error_text = await resp.text()
                        return ModelResponse(
                            content="",
                            model=model,
                            provider="ollama",
                            tokens_used=0,
                            latency=time.time() - start,
                            error=f"HTTP {resp.status}: {error_text}"
                        )
                        
        except Exception as e:
            logger.error(f"Ollama error: {e}")
            return ModelResponse(
                content="",
                model=model,
                provider="ollama",
                tokens_used=0,
                latency=0,
                error=str(e)
            )

class OpenAIClient:
    """Client for OpenAI"""
    
    def __init__(self, api_key: str, models: Dict[str, str] = None):
        self.api_key = api_key
        self.models = models or {}
    
    async def generate(self, prompt: str, model: str, system: Optional[str] = None,
                      temperature: float = 0.7, max_tokens: int = 4096) -> ModelResponse:
        """Generate response using OpenAI"""
        try:
            from openai import AsyncOpenAI
            
            start = time.time()
            client = AsyncOpenAI(api_key=self.api_key)
            
            messages = []
            if system:
                messages.append({"role": "system", "content": system})
            messages.append({"role": "user", "content": prompt})
            
            response = await client.chat.completions.create(
                model=model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens
            )
            
            latency = time.time() - start
            
            return ModelResponse(
                content=response.choices[0].message.content or "",
                model=model,
                provider="openai",
                tokens_used=response.usage.total_tokens,
                latency=latency
            )
            
        except Exception as e:
            logger.error(f"OpenAI error: {e}")
            return ModelResponse(
                content="",
                model=model,
                provider="openai",
                tokens_used=0,
                latency=0,
                error=str(e)
            )

class AnthropicClient:
    """Client for Anthropic Claude"""
    
    def __init__(self, api_key: str, models: Dict[str, str] = None):
        self.api_key = api_key
        self.models = models or {}
    
    async def generate(self, prompt: str, model: str, system: Optional[str] = None,
                      temperature: float = 0.7, max_tokens: int = 4096) -> ModelResponse:
        """Generate response using Claude"""
        try:
            from anthropic import AsyncAnthropic
            
            start = time.time()
            client = AsyncAnthropic(api_key=self.api_key)
            
            response = await client.messages.create(
                model=model,
                max_tokens=max_tokens,
                temperature=temperature,
                system=system or "",
                messages=[{"role": "user", "content": prompt}]
            )
            
            latency = time.time() - start
            
            return ModelResponse(
                content=response.content[0].text if response.content else "",
                model=model,
                provider="anthropic",
                tokens_used=response.usage.input_tokens + response.usage.output_tokens,
                latency=latency
            )
            
        except Exception as e:
            logger.error(f"Anthropic error: {e}")
            return ModelResponse(
                content="",
                model=model,
                provider="anthropic",
                tokens_used=0,
                latency=0,
                error=str(e)
            )

class NeuralRouter:
    """
    Neural Router - Intelligent Multi-LLM Orchestrator
    
    Automatically selects the best LLM for each task based on:
    - Task type and requirements
    - Model availability and performance
    - Cost optimization
    - Fallback chains
    """
    
    def __init__(self, config):
        self.config = config
        self.cache = ResponseCache(ttl=config.brain.cache_ttl)
        
        # Initialize directives manager
        try:
            from directives_manager import directives_manager
            self.directives_manager = directives_manager
            logger.info("Directives manager initialized")
        except Exception as e:
            logger.warning(f"Could not initialize directives manager: {e}")
            self.directives_manager = None
        
        # Initialize clients
        self.ollama = OllamaClient(
            host=config.ollama.host,
            models=config.ollama.models
        ) if config.ollama.enabled else None
        
        self.openai = OpenAIClient(
            api_key=config.openai.api_key,
            models=config.openai.models
        ) if config.openai.enabled and config.openai.api_key else None
        
        self.anthropic = AnthropicClient(
            api_key=config.anthropic.api_key,
            models=config.anthropic.models
        ) if config.anthropic.enabled and config.anthropic.api_key else None
        
        logger.info("Neural Router initialized")
        logger.info(f"Available providers: Ollama={self.ollama is not None}, "
                   f"OpenAI={self.openai is not None}, "
                   f"Anthropic={self.anthropic is not None}")
    
    def _select_model(self, task_ctx: TaskContext) -> Tuple[str, str, Any]:
        """
        Select the best model for the task
        Returns: (provider, model_name, client)
        """
        task_type = task_ctx.task_type
        
        # Priority routing based on task type
        if task_type in [TaskType.CODE_GENERATION, TaskType.CODE_DEBUG, TaskType.CODE_REVIEW]:
            # DeepSeek Coder is best for coding
            if self.ollama and "coder" in self.ollama.models:
                return ("ollama", self.ollama.models["coder"], self.ollama)
        
        if task_type == TaskType.REASONING:
            # DeepSeek R1 or Claude for reasoning
            if self.ollama and "reasoning" in self.ollama.models:
                return ("ollama", self.ollama.models["reasoning"], self.ollama)
            if self.anthropic:
                return ("anthropic", self.anthropic.models["sonnet"], self.anthropic)
        
        if task_ctx.requires_speed and task_type == TaskType.QUICK_TASK:
            # Fast local model
            if self.ollama and "fast" in self.ollama.models:
                return ("ollama", self.ollama.models["fast"], self.ollama)
        
        if task_type in [TaskType.CREATIVE, TaskType.PLANNING]:
            # Claude excels at creative and planning tasks
            if self.anthropic:
                return ("anthropic", self.anthropic.models["sonnet"], self.anthropic)
        
        if task_type == TaskType.CONVERSATION:
            # Use chat-optimized models
            if self.ollama and "chat" in self.ollama.models:
                return ("ollama", self.ollama.models["chat"], self.ollama)
        
        # Default fallback
        if self.ollama and "chat" in self.ollama.models:
            return ("ollama", self.ollama.models["chat"], self.ollama)
        if self.openai:
            return ("openai", self.openai.models["gpt4mini"], self.openai)
        if self.anthropic:
            return ("anthropic", self.anthropic.models["sonnet"], self.anthropic)
        
        raise RuntimeError("No LLM providers available")
    
    def _get_enhanced_system_prompt(self, task_ctx: TaskContext) -> Optional[str]:
        """
        Get enhanced system prompt with directives
        Directives are prepended to give them priority
        """
        base_system_prompt = task_ctx.system_prompt or ""
        
        # Get directives if manager is available
        if self.directives_manager:
            try:
                # Extract project name from metadata if available
                project_name = None
                if task_ctx.metadata and "project" in task_ctx.metadata:
                    project_name = task_ctx.metadata["project"]
                
                # Get active directives (global + project if specified)
                directives = self.directives_manager.get_active_directives(project_name)
                
                if directives:
                    # Prepend directives to system prompt
                    if base_system_prompt:
                        enhanced_prompt = f"{directives}\n\n---\n\n{base_system_prompt}"
                    else:
                        enhanced_prompt = directives
                    
                    logger.debug(f"Enhanced system prompt with directives for project: {project_name}")
                    return enhanced_prompt
                    
            except Exception as e:
                logger.warning(f"Error getting directives: {e}")
        
        # Fallback to base system prompt
        return base_system_prompt if base_system_prompt else None
    
    async def think(self, task_ctx: TaskContext) -> ModelResponse:
        """
        Process a task through the neural router with directives
        """
        # Get enhanced system prompt with directives
        enhanced_system_prompt = self._get_enhanced_system_prompt(task_ctx)
        
        # Check cache first
        if self.config.brain.cache_responses:
            provider, model, _ = self._select_model(task_ctx)
            cached = self.cache.get(task_ctx.prompt, task_ctx.task_type.value, f"{provider}:{model}")
            if cached:
                logger.info(f"Cache hit for {task_ctx.task_type.value}")
                return cached
        
        # Try primary model
        for attempt in range(self.config.brain.max_retries):
            try:
                provider, model, client = self._select_model(task_ctx)
                logger.info(f"Attempt {attempt + 1}: Using {provider}:{model} for {task_ctx.task_type.value}")
                
                response = await client.generate(
                    prompt=task_ctx.prompt,
                    model=model,
                    system=enhanced_system_prompt,  # Use enhanced prompt with directives
                    temperature=task_ctx.temperature,
                    max_tokens=task_ctx.max_tokens
                )
                
                if not response.error:
                    # Cache successful response
                    if self.config.brain.cache_responses:
                        self.cache.set(task_ctx.prompt, task_ctx.task_type.value, 
                                     f"{provider}:{model}", response)
                    return response
                
                logger.warning(f"Error from {provider}:{model} - {response.error}")
                
            except Exception as e:
                logger.error(f"Exception on attempt {attempt + 1}: {e}")
                if attempt == self.config.brain.max_retries - 1:
                    return ModelResponse(
                        content="",
                        model="none",
                        provider="none",
                        tokens_used=0,
                        latency=0,
                        error=f"All attempts failed: {str(e)}"
                    )
                await asyncio.sleep(1)  # Brief delay before retry
        
        # Should not reach here
        return ModelResponse(
            content="",
            model="none",
            provider="none",
            tokens_used=0,
            latency=0,
            error="All retry attempts exhausted"
        )
    
    async def think_sync(self, prompt: str, task_type: TaskType = TaskType.CONVERSATION,
                        system: Optional[str] = None) -> str:
        """
        Simplified synchronous-style think method
        Returns just the content string
        """
        task_ctx = TaskContext(
            prompt=prompt,
            task_type=task_type,
            system_prompt=system
        )
        response = await self.think(task_ctx)
        return response.content if not response.error else f"Error: {response.error}"
