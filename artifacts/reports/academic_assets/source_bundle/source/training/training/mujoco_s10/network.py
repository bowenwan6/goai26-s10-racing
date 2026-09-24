"""PyTorch actor-critic used by the native MuJoCo PPO trainer."""

from __future__ import annotations

import torch
from torch import nn


def _mlp(input_dim: int, hidden: tuple[int, ...], output_dim: int) -> nn.Sequential:
    sizes = (input_dim, *hidden, output_dim)
    layers: list[nn.Module] = []
    for index, (left, right) in enumerate(zip(sizes[:-1], sizes[1:])):
        linear = nn.Linear(left, right)
        nn.init.orthogonal_(linear.weight, gain=0.01 if index == len(sizes) - 2 else 1.414)
        nn.init.zeros_(linear.bias)
        layers.append(linear)
        if index != len(sizes) - 2:
            layers.append(nn.ELU())
    return nn.Sequential(*layers)


class GaussianActor(nn.Module):
    def __init__(self, observation_dim: int, hidden: tuple[int, ...]) -> None:
        super().__init__()
        self.mlp = _mlp(observation_dim, hidden, 16)
        self.log_std = nn.Parameter(torch.full((16,), -0.5))

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        return self.mlp(observation)

    def distribution(self, observation: torch.Tensor) -> torch.distributions.Normal:
        mean = self(observation)
        std = self.log_std.exp().expand_as(mean)
        return torch.distributions.Normal(mean, std)


class ActorCritic(nn.Module):
    def __init__(
        self,
        observation_dim: int,
        actor_hidden: tuple[int, ...] = (512, 256, 128),
        critic_hidden: tuple[int, ...] = (512, 256, 128),
    ) -> None:
        super().__init__()
        self.actor = GaussianActor(observation_dim, actor_hidden)
        self.critic = _mlp(observation_dim, critic_hidden, 1)

    @torch.no_grad()
    def act(self, observation: torch.Tensor):
        distribution = self.actor.distribution(observation)
        action = distribution.sample()
        log_prob = distribution.log_prob(action).sum(-1)
        value = self.critic(observation).squeeze(-1)
        return action, log_prob, value

    def evaluate(self, observation: torch.Tensor, action: torch.Tensor):
        distribution = self.actor.distribution(observation)
        log_prob = distribution.log_prob(action).sum(-1)
        entropy = distribution.entropy().sum(-1)
        value = self.critic(observation).squeeze(-1)
        return log_prob, entropy, value
