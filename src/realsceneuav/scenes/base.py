from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(slots=True)
class Target:
    target_id: str
    position: np.ndarray
    instruction: str
    metadata: dict[str, Any]


@dataclass(slots=True)
class Observation:
    rgb: np.ndarray | None = None
    depth: np.ndarray | None = None
    metadata: dict[str, Any] | None = None


class SceneAdapter(ABC):
    """Dataset/renderer boundary.

    A dataset adapter owns coordinate conversion, target definitions, and rendering.
    The flight loop does not know whether data came from CityNav, 3DGS, mesh, or a
    custom real-world capture.
    """

    @abstractmethod
    def scene_id(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def sample_targets(self) -> list[Target]:
        raise NotImplementedError

    @abstractmethod
    def observation(self, position: np.ndarray, rpy: np.ndarray) -> Observation:
        raise NotImplementedError

    @abstractmethod
    def ground_height(self, x: float, y: float) -> float:
        raise NotImplementedError

    def provenance(self) -> dict[str, Any]:
        """Serializable description of the scene/data source used by an episode."""

        return {
            "scene_id": self.scene_id(),
            "adapter": type(self).__name__,
        }
