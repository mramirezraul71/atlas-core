"""
ATLAS NEXUS - Autonomous Engine
Multi-Step Planning, Execution, and Self-Recovery System
"""

import asyncio
import json
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field
from enum import Enum
from datetime import datetime

from brain.neural_router import NeuralRouter, TaskContext, TaskType

logger = logging.getLogger("atlas.brain.autonomous")

class StepStatus(Enum):
    """Status of a plan step"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"

class PlanStatus(Enum):
    """Status of overall plan"""
    CREATED = "created"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    PAUSED = "paused"

@dataclass
class PlanStep:
    """Single step in an execution plan"""
    id: str
    description: str
    action: str
    parameters: Dict[str, Any]
    dependencies: List[str] = field(default_factory=list)
    status: StepStatus = StepStatus.PENDING
    result: Optional[Any] = None
    error: Optional[str] = None
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    retries: int = 0
    max_retries: int = 3

@dataclass
class ExecutionPlan:
    """Complete execution plan"""
    id: str
    goal: str
    steps: List[PlanStep]
    status: PlanStatus = PlanStatus.CREATED
    created_at: datetime = field(default_factory=datetime.now)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

class Planner:
    """
    AI-Powered Planner
    Breaks down complex goals into actionable steps
    """
    
    def __init__(self, router: NeuralRouter):
        self.router = router
    
    async def create_plan(self, goal: str, context: Dict[str, Any] = None) -> ExecutionPlan:
        """
        Create an execution plan for achieving a goal
        """
        logger.info(f"Creating plan for goal: {goal}")
        
        planning_prompt = f"""You are an AI planning system. Break down this goal into concrete, executable steps.

Goal: {goal}

Context: {json.dumps(context or {}, indent=2)}

Create a detailed execution plan. For each step provide:
- id: unique identifier (step_1, step_2, etc.)
- description: what this step does
- action: the action to execute (e.g., "search_web", "create_file", "analyze_data", "send_message")
- parameters: dict of parameters needed for the action
- dependencies: list of step ids that must complete first

Respond ONLY with valid JSON in this format:
{{
  "steps": [
    {{
      "id": "step_1",
      "description": "Search for information",
      "action": "search_web",
      "parameters": {{"query": "example"}},
      "dependencies": []
    }},
    ...
  ]
}}"""
        
        task_ctx = TaskContext(
            prompt=planning_prompt,
            task_type=TaskType.PLANNING,
            temperature=0.3,  # Lower temperature for consistent planning
            requires_accuracy=True
        )
        
        response = await self.router.think(task_ctx)
        
        try:
            # Parse the plan
            plan_data = json.loads(response.content)
            steps = []
            
            for step_data in plan_data.get("steps", []):
                step = PlanStep(
                    id=step_data["id"],
                    description=step_data["description"],
                    action=step_data["action"],
                    parameters=step_data.get("parameters", {}),
                    dependencies=step_data.get("dependencies", [])
                )
                steps.append(step)
            
            plan = ExecutionPlan(
                id=f"plan_{int(datetime.now().timestamp())}",
                goal=goal,
                steps=steps,
                metadata={"context": context}
            )
            
            logger.info(f"Created plan with {len(steps)} steps")
            return plan
            
        except Exception as e:
            logger.error(f"Failed to parse plan: {e}")
            logger.error(f"Response was: {response.content}")
            
            # Create a simple fallback plan
            fallback_step = PlanStep(
                id="step_1",
                description=f"Execute goal: {goal}",
                action="generic_task",
                parameters={"goal": goal}
            )
            
            return ExecutionPlan(
                id=f"plan_{int(datetime.now().timestamp())}",
                goal=goal,
                steps=[fallback_step]
            )

class Executor:
    """
    Plan Executor
    Executes plans step by step with dependency management
    """
    
    def __init__(self, router: NeuralRouter, tool_registry):
        self.router = router
        self.tool_registry = tool_registry
    
    async def execute_step(self, step: PlanStep, plan_context: Dict[str, Any]) -> bool:
        """
        Execute a single step
        Returns True if successful, False otherwise
        """
        logger.info(f"Executing step: {step.id} - {step.description}")
        
        step.status = StepStatus.RUNNING
        step.started_at = datetime.now()
        
        try:
            # Get the tool for this action
            tool = self.tool_registry.get_tool(step.action)
            
            if tool:
                # Execute using registered tool
                result = await tool.execute(step.parameters, plan_context)
                step.result = result
                step.status = StepStatus.COMPLETED
                step.completed_at = datetime.now()
                logger.info(f"Step {step.id} completed successfully")
                return True
            else:
                # Fallback: Ask AI to help
                logger.warning(f"No tool found for action '{step.action}', using AI fallback")
                
                fallback_prompt = f"""Execute this task:
Action: {step.action}
Description: {step.description}
Parameters: {json.dumps(step.parameters, indent=2)}
Context: {json.dumps(plan_context, indent=2)}

Provide the result or explain what would need to be done."""
                
                task_ctx = TaskContext(
                    prompt=fallback_prompt,
                    task_type=TaskType.ANALYSIS,
                    temperature=0.5
                )
                
                response = await self.router.think(task_ctx)
                step.result = response.content
                step.status = StepStatus.COMPLETED
                step.completed_at = datetime.now()
                return True
                
        except Exception as e:
            logger.error(f"Step {step.id} failed: {e}")
            step.error = str(e)
            step.retries += 1
            
            if step.retries < step.max_retries:
                logger.info(f"Retrying step {step.id} ({step.retries}/{step.max_retries})")
                await asyncio.sleep(2 ** step.retries)  # Exponential backoff
                return await self.execute_step(step, plan_context)
            else:
                step.status = StepStatus.FAILED
                step.completed_at = datetime.now()
                return False
    
    async def execute_plan(self, plan: ExecutionPlan) -> ExecutionPlan:
        """
        Execute complete plan with dependency management
        """
        logger.info(f"Starting execution of plan: {plan.id}")
        
        plan.status = PlanStatus.RUNNING
        plan.started_at = datetime.now()
        
        completed_steps = set()
        plan_context = plan.metadata.copy()
        
        while True:
            # Find steps ready to execute (dependencies met)
            ready_steps = [
                step for step in plan.steps
                if step.status == StepStatus.PENDING
                and all(dep in completed_steps for dep in step.dependencies)
            ]
            
            if not ready_steps:
                # Check if all steps are done
                if all(step.status in [StepStatus.COMPLETED, StepStatus.SKIPPED, StepStatus.FAILED] 
                      for step in plan.steps):
                    break
                else:
                    # Deadlock - some dependencies can't be met
                    logger.error("Plan execution deadlocked - unmet dependencies")
                    plan.status = PlanStatus.FAILED
                    break
            
            # Execute ready steps
            for step in ready_steps:
                success = await self.execute_step(step, plan_context)
                
                if success:
                    completed_steps.add(step.id)
                    # Update context with step result
                    plan_context[f"step_{step.id}_result"] = step.result
                else:
                    # Step failed - decide whether to continue
                    logger.warning(f"Step {step.id} failed - continuing with remaining steps")
        
        # Determine final status
        failed_steps = [s for s in plan.steps if s.status == StepStatus.FAILED]
        if failed_steps:
            plan.status = PlanStatus.FAILED
            logger.warning(f"Plan failed - {len(failed_steps)} steps failed")
        else:
            plan.status = PlanStatus.COMPLETED
            logger.info("Plan completed successfully")
        
        plan.completed_at = datetime.now()
        return plan

class AutonomousEngine:
    """
    Main Autonomous Engine
    Combines planning and execution with self-recovery
    """
    
    def __init__(self, config, router: NeuralRouter, tool_registry):
        self.config = config
        self.router = router
        self.tool_registry = tool_registry
        
        self.planner = Planner(router)
        self.executor = Executor(router, tool_registry)
        
        self.active_plans: Dict[str, ExecutionPlan] = {}
        
        logger.info("Autonomous Engine initialized")
    
    async def achieve_goal(self, goal: str, context: Dict[str, Any] = None) -> ExecutionPlan:
        """
        Autonomously achieve a goal through planning and execution
        """
        logger.info(f"🎯 New goal: {goal}")
        
        # Create plan
        plan = await self.planner.create_plan(goal, context)
        self.active_plans[plan.id] = plan
        
        # Execute plan
        executed_plan = await self.executor.execute_plan(plan)
        
        # Check if we need recovery
        if executed_plan.status == PlanStatus.FAILED:
            if self.config.autonomy.auto_recovery:
                logger.info("🔄 Attempting auto-recovery")
                recovered_plan = await self.attempt_recovery(executed_plan)
                return recovered_plan
        
        return executed_plan
    
    async def attempt_recovery(self, failed_plan: ExecutionPlan) -> ExecutionPlan:
        """
        Attempt to recover from a failed plan
        """
        logger.info(f"Attempting recovery for plan: {failed_plan.id}")
        
        # Analyze what went wrong
        failed_steps = [s for s in failed_plan.steps if s.status == StepStatus.FAILED]
        
        recovery_prompt = f"""A plan failed. Analyze what went wrong and create a recovery plan.

Original Goal: {failed_plan.goal}

Failed Steps:
{json.dumps([{
    'id': s.id,
    'description': s.description,
    'error': s.error
} for s in failed_steps], indent=2)}

Create a recovery plan that:
1. Identifies the root cause
2. Proposes alternative approaches
3. Provides specific steps to try again

Respond with a new goal statement that incorporates the recovery strategy."""
        
        task_ctx = TaskContext(
            prompt=recovery_prompt,
            task_type=TaskType.REASONING,
            temperature=0.4
        )
        
        response = await self.router.think(task_ctx)
        recovery_goal = response.content
        
        # Create and execute recovery plan
        logger.info(f"Recovery goal: {recovery_goal}")
        recovery_plan = await self.achieve_goal(recovery_goal, failed_plan.metadata)
        
        return recovery_plan
    
    def get_plan_status(self, plan_id: str) -> Optional[ExecutionPlan]:
        """Get status of a plan"""
        return self.active_plans.get(plan_id)
    
    def list_active_plans(self) -> List[ExecutionPlan]:
        """List all active plans"""
        return list(self.active_plans.values())
