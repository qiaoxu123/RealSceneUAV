from __future__ import annotations

import numpy as np

from realsceneuav.scenes.base import SceneAdapter
from realsceneuav.tasks.navigation import NavigationTask


def sample_task(scene: SceneAdapter, seed: int = 0) -> NavigationTask:
    rng = np.random.default_rng(seed)
    targets = scene.sample_targets()
    if not targets:
        raise RuntimeError("Scene exposes no targets")
    target = targets[int(rng.integers(0, len(targets)))]

    offset = np.array([-40.0, -20.0, 15.0], dtype=np.float64)
    start = target.position + offset
    return NavigationTask(
        episode_id=f"{scene.scene_id()}-{seed:06d}",
        start_position=start,
        start_yaw=0.0,
        target=target,
    )
