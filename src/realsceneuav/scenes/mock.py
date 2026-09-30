from __future__ import annotations

import numpy as np

from realsceneuav.scenes.base import Observation, SceneAdapter, Target


class MockRealScene(SceneAdapter):
    """Tiny deterministic scene for CI and interface tests."""

    def scene_id(self) -> str:
        return "mock-real-scene"

    def sample_targets(self) -> list[Target]:
        return [
            Target(
                target_id="building-001",
                position=np.array([30.0, 10.0, 5.0], dtype=np.float64),
                instruction="Fly to the building near the road.",
                metadata={"category": "building"},
            )
        ]

    def observation(self, position: np.ndarray, rpy: np.ndarray) -> Observation:
        rgb = np.zeros((64, 64, 3), dtype=np.uint8)
        depth = np.full((64, 64), 50.0, dtype=np.float32)
        pose = [float(value) for value in np.concatenate([position, rpy])]
        return Observation(rgb=rgb, depth=depth, metadata={"pose": pose})

    def ground_height(self, x: float, y: float) -> float:
        return 0.0
