"""
POT: Simulation & Training Operations
======================================
Procedimientos para simulación física y entrenamiento RL.

Triggers:
- Inicio de entrenamiento
- Evaluación de política
- Transferencia sim-to-real
- Validación de modelo

Severidad: MEDIUM
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="simulation_training",
        name="Simulación y Entrenamiento",
        description="""
Procedimiento para operaciones de simulación:
1. Inicializar motor de simulación
2. Cargar modelo del robot
3. Configurar domain randomization
4. Ejecutar entrenamiento/evaluación
5. Preparar transferencia sim-to-real
        """.strip(),
        category=POTCategory.MAINTENANCE,
        severity=POTSeverity.MEDIUM,
        version="1.0.0",
        author="ATLAS Robotics Architect",
        
        trigger_check_ids=["simulation_*", "training_*", "sim2real_*"],
        trigger_keywords=["simulación", "entrenar", "mujoco", "sim2real", "RL", "policy"],
        
        prerequisites=[
            "Python con numpy disponible",
            "Opcionalmente: MuJoCo, PyBullet, o Isaac Sim",
        ],
        required_services=[],
        required_permissions=[],
        
        objectives=[
            "Inicializar entorno de simulación",
            "Cargar y validar modelo del robot",
            "Aplicar domain randomization",
            "Ejecutar episodios de entrenamiento",
            "Evaluar brecha sim-to-real",
        ],
        success_criteria="Política entrenada y lista para transferencia",
        estimated_duration_minutes=60,
        
        tutorial_overview="""
## Guía de Simulación ATLAS

### Arquitectura
```
┌──────────────────┐     ┌──────────────────┐
│  SimulationEngine │────▶│  TrainingEnv     │
│   (MuJoCo/PB)    │     │  (Gymnasium)     │
└──────────────────┘     └──────────────────┘
         │                        │
         ▼                        ▼
┌──────────────────┐     ┌──────────────────┐
│  RobotModel      │     │  Policy (RL)     │
│  (URDF/MJCF)     │     │  (PPO/SAC)       │
└──────────────────┘     └──────────────────┘
         │                        │
         ▼                        ▼
┌──────────────────┐     ┌──────────────────┐
│  DomainRandom    │     │  Sim2Real        │
│  (params)        │     │  (transfer)      │
└──────────────────┘     └──────────────────┘
```

### Flujo de Trabajo
1. **Configurar**: Crear SimConfig y EntornoConfig
2. **Cargar**: Cargar modelo URDF/MJCF del robot
3. **Entrenar**: Ejecutar episodios con RL
4. **Randomizar**: Variar parámetros para robustez
5. **Transferir**: Aplicar a hardware real

### Backends Soportados
- `mujoco`: MuJoCo (recomendado)
- `pybullet`: PyBullet
- `isaac`: NVIDIA Isaac Sim
- `internal`: Simulador básico interno

### Uso Básico
```python
from simulation import SimulationEngine, TrainingEnvironment

# Crear motor
sim = SimulationEngine()
sim.initialize()
sim.load_robot("models/atlas_humanoid.urdf")

# Crear entorno
env = TrainingEnvironment()
obs, info = env.reset()

# Entrenamiento
for _ in range(1000):
    action = policy(obs)
    obs, reward, done, truncated, info = env.step(action)
    if done:
        obs, info = env.reset()
```

### Domain Randomization
```python
from simulation import DomainRandomizer

dr = DomainRandomizer()
params = dr.randomize()
# params = {
#   "gravity_z": -9.5,
#   "friction": 1.2,
#   "mass_scale": 0.95,
#   ...
# }
```

### Transferencia Sim2Real
```python
from simulation import Sim2RealTransfer

transfer = Sim2RealTransfer()
adapted_policy = transfer.prepare_policy(trained_policy)

# En el robot real
transfer.collect_real_data(state, action, next_state)
```
        """,
        
        steps=[
            POTStep(
                id="init_simulation",
                name="Inicializar simulación",
                description="Crear motor de simulación",
                step_type=StepType.COMMAND,
                command='python -c "from simulation import SimulationEngine; s=SimulationEngine(); print(s.initialize())"',
                timeout_seconds=30,
                capture_output=True,
            ),
            POTStep(
                id="load_robot_model",
                name="Cargar modelo del robot",
                description="Cargar URDF/MJCF del humanoid",
                step_type=StepType.COMMAND,
                command='python -c "from simulation import RobotModel; m=RobotModel(); print(m.to_dict())"',
                timeout_seconds=15,
                capture_output=True,
                tutorial_notes="El modelo define joints, links y dinámica",
            ),
            POTStep(
                id="setup_domain_random",
                name="Configurar Domain Randomization",
                description="Establecer rangos de variación",
                step_type=StepType.LOG,
                tutorial_notes="""
Domain Randomization mejora la robustez:
- Variar gravedad, fricción, masas
- Variar iluminación, texturas
- Variar ruido de sensores
                """,
            ),
            POTStep(
                id="create_training_env",
                name="Crear entorno de entrenamiento",
                description="Inicializar entorno Gymnasium",
                step_type=StepType.COMMAND,
                command='python -c "from simulation import TrainingEnvironment; e=TrainingEnvironment(); print(e.get_info())"',
                timeout_seconds=15,
                capture_output=True,
            ),
            POTStep(
                id="run_episode",
                name="Ejecutar episodio de prueba",
                description="Verificar funcionamiento",
                step_type=StepType.COMMAND,
                command='python -c "from simulation import SimulationEngine; s=SimulationEngine(); s.initialize(); r=s.run_episode(max_steps=100); print(r)"',
                timeout_seconds=60,
                capture_output=True,
            ),
            POTStep(
                id="evaluate_sim2real",
                name="Evaluar brecha sim2real",
                description="Comparar simulación vs realidad",
                step_type=StepType.LOG,
                tutorial_notes="""
Métricas de brecha:
- Error de posición
- Error de velocidad
- Diferencia de reward
                """,
            ),
            POTStep(
                id="log_results",
                name="Registrar resultados",
                description="Guardar métricas de entrenamiento",
                step_type=StepType.NOTIFY,
                notify_message="Simulación/entrenamiento completado",
            ),
        ],
        
        has_rollback=False,
        tags=["simulation", "training", "RL", "sim2real", "mujoco"],
    )
