"""ATLAS humanoid: kernel + brain, hands, eyes, ears, autonomy, comms, update."""
from __future__ import annotations

from modules.humanoid.kernel import Kernel
from modules.humanoid.brain import BrainOrchestrator
from modules.humanoid.hands import HandsModule
from modules.humanoid.eyes import EyesModule
from modules.humanoid.ears import EarsModule
from modules.humanoid.autonomy import AutonomyModule
from modules.humanoid.comms import CommsModule
from modules.humanoid.update import UpdateModule


def create_humanoid_kernel() -> Kernel:
    """Build kernel and register all modules. Brain wired to Autonomy."""
    k = Kernel()
    brain = BrainOrchestrator()
    k.register(brain)
    k.register(HandsModule())
    k.register(EyesModule())
    k.register(EarsModule())
    autonomy = AutonomyModule()
    autonomy.set_brain(brain)
    k.register(autonomy)
    k.register(CommsModule())
    k.register(UpdateModule())
    return k


_humanoid_kernel: Kernel | None = None


def get_humanoid_kernel() -> Kernel:
    global _humanoid_kernel
    if _humanoid_kernel is None:
        _humanoid_kernel = create_humanoid_kernel()
    return _humanoid_kernel
