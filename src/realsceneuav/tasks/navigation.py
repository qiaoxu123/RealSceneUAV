from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from realsceneuav.scenes.base import Target


@dataclass(slots=True)
class NavigationTask:
    episode_id: str
    start_position: np.ndarray
    start_yaw: float
    target: Target
    success_radius_m: float = 5.0
    max_duration_s: float = 300.0

    def distance_to_target(self, position: np.ndarray) -> float:
        return float(np.linalg.norm(position - self.target.position))

    def success(self, position: np.ndarray) -> bool:
        return self.distance_to_target(position) <= self.success_radius_m
