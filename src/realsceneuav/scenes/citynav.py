from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from realsceneuav.scenes.base import Observation, SceneAdapter, Target


class CityNavTrajectoryAdapter(SceneAdapter):
    """Metadata adapter for released CityNav MTurk trajectory JSON files.

    It deliberately does not assume a renderer. The released trajectory files provide
    target positions, descriptions, areas/blocks and human trajectories. RGB/depth
    rendering should be attached separately from SensatUrban raster data or another
    real-scene renderer.
    """

    def __init__(self, trajectory_json: str | Path, ground_height_m: float = 0.0) -> None:
        self.path = Path(trajectory_json)
        self.ground_height_m = float(ground_height_m)
        self.records = json.loads(self.path.read_text())
        if not isinstance(self.records, list):
            raise ValueError("CityNav trajectory JSON must contain a list of episodes")

    def scene_id(self) -> str:
        return self.path.stem

    def sample_targets(self) -> list[Target]:
        targets: list[Target] = []
        for index, record in enumerate(self.records):
            positions = record.get("target_positions") or []
            descriptions = record.get("descriptions") or []
            object_ids = record.get("object_ids") or [index]
            if not positions:
                continue
            position = np.asarray(positions[-1], dtype=np.float64)
            instruction = str(descriptions[0]) if descriptions else ""
            targets.append(
                Target(
                    target_id=str(object_ids[0]),
                    position=position,
                    instruction=instruction,
                    metadata={
                        "area": record.get("area"),
                        "block": record.get("block"),
                        "ann_ids": record.get("ann_ids", []),
                        "split": record.get("split"),
                        "source_index": index,
                    },
                )
            )
        return targets

    def observation(self, position: np.ndarray, rpy: np.ndarray) -> Observation:
        raise NotImplementedError(
            "CityNav metadata is loaded, but visual rendering requires a SensatUrban "
            "RGB-D/point-cloud/3DGS renderer backend."
        )

    def ground_height(self, x: float, y: float) -> float:
        return self.ground_height_m

    def human_trajectory(self, source_index: int) -> np.ndarray:
        trajectory = self.records[source_index].get("trajectory") or []
        return np.asarray(trajectory, dtype=np.float64)
