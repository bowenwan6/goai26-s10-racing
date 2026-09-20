"""RSL-RL checkpoint helpers shared by warm-start and ONNX export.

The pinned DeepRobotics trainer can encounter two checkpoint layouts:

* RSL-RL 3.1+ stores the actor under ``actor_state_dict`` with ``mlp.*`` keys;
* older releases store it under ``model_state_dict`` with ``actor.*`` keys.

Deployment only needs the deterministic MLP mean.  These helpers reconstruct that MLP
without starting Isaac Sim, while refusing normalization layouts that would otherwise be
silently omitted from the exported policy.
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass

import torch
from torch import nn

_LINEAR_KEY = re.compile(
    r"^(?P<prefix>(?:_orig_mod\.)?(?:mlp|actor))\.(?P<index>\d+)\.weight$"
)


@dataclass(frozen=True)
class ActorState:
    """An actor state dict plus the checkpoint container it came from."""

    state_dict: Mapping[str, torch.Tensor]
    container_key: str
    prefix: str


def extract_actor_state(checkpoint: Mapping[str, object]) -> ActorState:
    """Return the actor tensors from a current or legacy RSL-RL checkpoint."""
    for container_key in ("actor_state_dict", "model_state_dict"):
        candidate = checkpoint.get(container_key)
        if not isinstance(candidate, Mapping):
            continue

        prefixes = {
            match.group("prefix")
            for key in candidate
            if isinstance(key, str) and (match := _LINEAR_KEY.match(key))
        }
        if len(prefixes) != 1:
            raise ValueError(
                f"{container_key} must contain one actor MLP prefix, found {sorted(prefixes)}"
            )
        return ActorState(candidate, container_key, prefixes.pop())

    raise ValueError(
        "checkpoint has no supported actor_state_dict or model_state_dict; "
        f"found keys {sorted(checkpoint)}"
    )


def linear_layer_keys(actor: ActorState) -> list[tuple[str, str]]:
    """Return ``(weight_key, bias_key)`` pairs in forward order."""
    indexed: list[tuple[int, str, str]] = []
    prefix = re.escape(actor.prefix)
    pattern = re.compile(rf"^{prefix}\.(?P<index>\d+)\.weight$")
    for key in actor.state_dict:
        if not isinstance(key, str) or not (match := pattern.match(key)):
            continue
        bias_key = key.removesuffix("weight") + "bias"
        if bias_key not in actor.state_dict:
            raise ValueError(f"actor layer {key} has no matching {bias_key}")
        indexed.append((int(match.group("index")), key, bias_key))

    if not indexed:
        raise ValueError("actor state contains no linear layers")
    return [(weight, bias) for _, weight, bias in sorted(indexed)]


def policy_dimensions(actor: ActorState) -> tuple[int, int]:
    """Infer deterministic actor input and output widths."""
    layers = linear_layer_keys(actor)
    first = actor.state_dict[layers[0][0]]
    last = actor.state_dict[layers[-1][0]]
    if first.ndim != 2 or last.ndim != 2:
        raise ValueError("actor linear weights must be matrices")
    return int(first.shape[1]), int(last.shape[0])


def build_actor(actor: ActorState) -> nn.Sequential:
    """Reconstruct the deterministic ELU MLP encoded by an RSL-RL actor state."""
    normalizer_keys = [key for key in actor.state_dict if "obs_normalizer" in str(key)]
    if normalizer_keys:
        raise ValueError(
            "checkpoint uses observation normalization; export it through the live RSL-RL runner"
        )

    pairs = linear_layer_keys(actor)
    modules: list[nn.Module] = []
    for layer_index, (weight_key, bias_key) in enumerate(pairs):
        weight = actor.state_dict[weight_key]
        bias = actor.state_dict[bias_key]
        if weight.ndim != 2 or bias.ndim != 1 or bias.shape[0] != weight.shape[0]:
            raise ValueError(f"invalid linear tensors {weight_key}/{bias_key}")
        linear = nn.Linear(weight.shape[1], weight.shape[0])
        with torch.no_grad():
            linear.weight.copy_(weight)
            linear.bias.copy_(bias)
        modules.append(linear)
        if layer_index != len(pairs) - 1:
            modules.append(nn.ELU())

    return nn.Sequential(*modules).eval()
