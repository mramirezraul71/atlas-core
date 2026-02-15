"""
Model-Agnostic Meta-Learning (MAML) - Finn et al.
Adaptación rápida a nuevas tareas con pocos ejemplos (few-shot).
"""
from __future__ import annotations

from collections import OrderedDict
from typing import Any, Dict, List, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
    from torch import optim
    _TORCH_AVAILABLE = True
except ImportError:
    torch = None
    nn = None
    optim = None
    _TORCH_AVAILABLE = False


if _TORCH_AVAILABLE:

    class MAMLPolicy(nn.Module):
        """Policy network para meta-learning."""

        def __init__(
            self,
            state_dim: int = 128,
            action_dim: int = 64,
            hidden_dim: int = 256,
        ):
            super().__init__()
            self.network = nn.Sequential(
                nn.Linear(state_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, action_dim),
            )

        def forward(self, state: "torch.Tensor") -> "torch.Tensor":
            return self.network(state)

else:
    MAMLPolicy = None  # type: ignore

if _TORCH_AVAILABLE:

    class MAML:
        """
        Meta-Learning para adaptación rápida a nuevas tareas.
        Entrena sobre una distribución de tareas y se adapta en pocos pasos.
        """

        def __init__(
        self,
            state_dim: int = 128,
            action_dim: int = 64,
            meta_lr: float = 1e-3,
            inner_lr: float = 1e-2,
            inner_steps: int = 5,
        ):
            self.state_dim = state_dim
            self.action_dim = action_dim
            self.policy = MAMLPolicy(state_dim, action_dim)
            self.meta_optimizer = optim.Adam(self.policy.parameters(), lr=meta_lr)
            self.inner_lr = inner_lr
            self.inner_steps = inner_steps
            self.task_buffer = []

        def add_task(self, task_data: Dict[str, Any]) -> None:
            self.task_buffer.append(task_data)

        def _sample_batch(self, data: List[Tuple], batch_size: int) -> List[Tuple]:
            if len(data) <= batch_size:
                return list(data)
            indices = np.random.choice(len(data), batch_size, replace=False)
            return [data[i] for i in indices]

        def _forward_with_params(self, states, params: OrderedDict):
            param_list = list(params.values())
            x = states
            n = len(param_list)
            for i in range(0, n, 2):
                weight = param_list[i]
                bias = param_list[i + 1] if i + 1 < n else None
                x = torch.matmul(x, weight.t())
                if bias is not None:
                    x = x + bias
                if i < n - 2:
                    x = torch.relu(x)
            return x

        def inner_loop(self, task_data: Dict[str, Any]) -> Tuple[OrderedDict, float]:
            adapted_params = OrderedDict(self.policy.named_parameters())
            for step in range(self.inner_steps):
                batch = self._sample_batch(task_data["demonstrations"], batch_size=10)
                states, actions, rewards = zip(*batch)
                states = torch.FloatTensor(np.array(states))
                actions = torch.FloatTensor(np.array(actions))
                rewards = torch.FloatTensor(np.array(rewards))
                predictions = self._forward_with_params(states, adapted_params)
                loss = nn.MSELoss()(predictions, actions) * (1.0 + rewards.mean())
                grads = torch.autograd.grad(
                    loss, adapted_params.values(), create_graph=True, allow_unused=True
                )
                adapted_params = OrderedDict(
                    (name, param - self.inner_lr * (grad if grad is not None else torch.zeros_like(param)))
                    for ((name, param), grad) in zip(adapted_params.items(), grads)
                )
            return adapted_params, loss.item()

        def meta_train_step(self, num_tasks: int = 8) -> Dict[str, Any]:
            if len(self.task_buffer) < num_tasks:
                return {"error": "Not enough tasks for meta-training", "meta_loss": None}
            sampled = list(np.random.choice(len(self.task_buffer), num_tasks, replace=False))
            sampled_tasks = [self.task_buffer[i] for i in sampled]
            meta_loss = 0.0
            task_losses = []
            for task in sampled_tasks:
                adapted_params, _ = self.inner_loop(task)
                val_batch = self._sample_batch(task["validation_data"], batch_size=10)
                val_states, val_actions, _ = zip(*val_batch)
                val_states = torch.FloatTensor(np.array(val_states))
                val_actions = torch.FloatTensor(np.array(val_actions))
                val_predictions = self._forward_with_params(val_states, adapted_params)
                task_loss = nn.MSELoss()(val_predictions, val_actions)
                meta_loss = meta_loss + task_loss
                task_losses.append(task_loss.item())
            meta_loss = meta_loss / num_tasks
            self.meta_optimizer.zero_grad()
            meta_loss.backward()
            self.meta_optimizer.step()
            return {
                "meta_loss": meta_loss.item(),
                "task_losses": task_losses,
                "avg_task_loss": float(np.mean(task_losses)),
                "num_tasks_trained": num_tasks,
            }

        def adapt_to_new_task(self, demonstrations: List[Tuple], num_adaptation_steps: int = 5):
            task_data = {"demonstrations": demonstrations, "validation_data": demonstrations}
            adapted_params, _ = self.inner_loop(task_data)
            vals = list(adapted_params.values())
            state_dim = int(vals[0].shape[1])
            action_dim = int(vals[-1].shape[0])
            adapted_policy = MAMLPolicy(state_dim=state_dim, action_dim=action_dim)
            adapted_policy.load_state_dict(adapted_params, strict=False)
            return adapted_policy

        def save_checkpoint(self, path: str) -> None:
            torch.save(
                {
                    "policy_state_dict": self.policy.state_dict(),
                    "optimizer_state_dict": self.meta_optimizer.state_dict(),
                    "task_buffer": self.task_buffer,
                },
                path,
            )

        def load_checkpoint(self, path: str) -> None:
            checkpoint = torch.load(path, map_location="cpu", weights_only=False)
            self.policy.load_state_dict(checkpoint["policy_state_dict"])
            self.meta_optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
            self.task_buffer = checkpoint.get("task_buffer", [])

else:
    MAML = None  # type: ignore
