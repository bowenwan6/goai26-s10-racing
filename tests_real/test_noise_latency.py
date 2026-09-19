from __future__ import annotations

import numpy as np
import pytest
from conftest import snapshot

from real_transfer.shadow import ShadowSession


@pytest.mark.parametrize("seed", range(10))
@pytest.mark.parametrize("latency", [0.02, 0.10])
def test_fixed_seed_sensor_noise_and_coherent_latency(config, seed, latency):
    rng = np.random.default_rng(seed)
    session = ShadowSession(config)
    computed = 0
    for i in range(100):
        data = snapshot(i * 0.02)
        # Coherent acquisition latency: all timestamps remain increasing.
        delay = latency + rng.uniform(-0.002, 0.002)
        for item in data["inputs"].values():
            item["stamp"] -= delay
        p = np.array(data["inputs"]["cloud"]["points"])
        p[:, 2] += rng.uniform(-0.01, 0.01, len(p))
        data["inputs"]["cloud"]["points"] = p.tolist()
        result = session.step(data)
        computed += result["candidate_computed"]
        assert result["transport_command"] == [0, 0, 0]
        assert np.linalg.norm(result["candidate"][:2]) <= 0.2 + 1e-12
    assert computed >= 40, result["reasons"]


def test_300_ms_latency_rejects_otherwise_perfect_inputs(config):
    session = ShadowSession(config)
    for i in range(100):
        data = snapshot(i * 0.02)
        for item in data["inputs"].values():
            item["stamp"] -= 0.3
        result = session.step(data)
        assert not result["candidate_computed"]
        assert "source_age_pose" in result["reasons"]
