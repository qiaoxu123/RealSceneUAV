from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from realsceneuav.scenes.base import Observation, SceneAdapter, Target
from realsceneuav.tasks.navigation import NavigationTask


class CityNavTrajectoryAdapter(SceneAdapter):
    """Adapter for released CityNav MTurk trajectory JSON files.

    Historical CityNav poses are normalized to [x, y, z, yaw, pitch].
    Rendering remains a separate real-scene responsibility.
    """

    def __init__(self, trajectory_json: str | Path, ground_height_m: float = 0.0) -> None:
        self.path = Path(trajectory_json)
        self.ground_height_m = float(ground_height_m)
        self.records: list[dict[str, Any]] = json.loads(self.path.read_text())
        if not isinstance(self.records, list):
            raise ValueError("CityNav trajectory JSON must contain a list of episodes")

    def scene_id(self) -> str:
        return self.path.stem

    @staticmethod
    def _map_name(record: dict[str, Any]) -> str:
        return f"{record['area']}_block_{record['block']}"

    @staticmethod
    def _pose5d(raw_pose: list[float] | tuple[float, ...]) -> np.ndarray:
        values = np.asarray(raw_pose, dtype=np.float64)
        if values.shape == (5,):
            return values
        if values.shape == (6,):
            x, y, z, dx, dy, dz = values
            yaw = np.arctan2(dy, dx)
            pitch = np.arctan2(dz, np.hypot(dx, dy))
            return np.asarray([x, y, z, yaw, pitch], dtype=np.float64)
        raise ValueError(f"Unsupported CityNav pose shape: {values.shape}")

    def map_name(self, source_index: int) -> str:
        return self._map_name(self.records[source_index])

    def source_record(self, source_index: int) -> dict[str, Any]:
        return self.records[source_index]

    def source_indices_for_map(self, map_name: str) -> list[int]:
        return [
            index
            for index, record in enumerate(self.records)
            if self._map_name(record) == map_name
        ]

    def target_for_record(self, source_index: int) -> Target:
        record = self.records[source_index]
        positions = record.get("target_positions") or []
        descriptions = record.get("descriptions") or []
        object_ids = record.get("object_ids") or [source_index]
        if not positions:
            raise ValueError(f"CityNav record {source_index} has no target_positions")

        return Target(
            target_id=str(object_ids[0]),
            position=np.asarray(positions[-1], dtype=np.float64),
            instruction=str(descriptions[0]) if descriptions else "",
            metadata={
                "area": record.get("area"),
                "block": record.get("block"),
                "map_name": self._map_name(record),
                "ann_ids": record.get("ann_ids", []),
                "split": record.get("split"),
                "source_index": source_index,
                "marker_positions": record.get("marker_positions", []),
                "dist_marker_to_target": record.get("dist_marker_to_target"),
                "dist_start_to_target": record.get("dist_start_to_target"),
            },
        )

    def sample_targets(self) -> list[Target]:
        targets: list[Target] = []
        for index, record in enumerate(self.records):
            if record.get("target_positions"):
                targets.append(self.target_for_record(index))
        return targets

    def human_trajectory(self, source_index: int) -> np.ndarray:
        """Return the original trajectory without changing its encoding."""

        trajectory = self.records[source_index].get("trajectory") or []
        return np.asarray(trajectory, dtype=np.float64)

    def human_pose_trajectory(self, source_index: int) -> np.ndarray:
        """Return an N x 5 trajectory: x, y, z, yaw, pitch."""

        trajectory = self.records[source_index].get("trajectory") or []
        if not trajectory:
            return np.empty((0, 5), dtype=np.float64)
        return np.stack([self._pose5d(pose) for pose in trajectory])

    def navigation_task(
        self,
        source_index: int,
        success_radius_m: float = 5.0,
        max_duration_s: float = 300.0,
    ) -> NavigationTask:
        """Reconstruct one navigation task from a released human episode."""

        poses = self.human_pose_trajectory(source_index)
        if len(poses) == 0:
            raise ValueError(f"CityNav record {source_index} has no trajectory")

        start = poses[0]
        target = self.target_for_record(source_index)
        return NavigationTask(
            episode_id=f"citynav-{self.map_name(source_index)}-{source_index:06d}",
            start_position=start[:3].copy(),
            start_yaw=float(start[3]),
            target=target,
            success_radius_m=success_radius_m,
            max_duration_s=max_duration_s,
        )

    def observation(self, position: np.ndarray, rpy: np.ndarray) -> Observation:
        raise NotImplementedError(
            "CityNav trajectory metadata does not contain visual pixels. Attach a "
            "CityNavRasterScene or another real-scene renderer."
        )

    def ground_height(self, x: float, y: float) -> float:
        return self.ground_height_m
