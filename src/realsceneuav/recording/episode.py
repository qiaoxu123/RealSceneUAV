from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.scenes.base import Observation
from realsceneuav.tasks.navigation import NavigationTask


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


class EpisodeRecorder:
    """Write task metadata and synchronized control/state/observation samples."""

    def __init__(self, output_root: str | Path, task: NavigationTask) -> None:
        self.root = Path(output_root) / task.episode_id
        self.root.mkdir(parents=True, exist_ok=False)

        self.rgb_dir = self.root / "rgb"
        self.depth_dir = self.root / "depth"
        self.rgb_dir.mkdir()
        self.depth_dir.mkdir()

        self._trajectory_fp = (self.root / "trajectory.csv").open("w", newline="")
        self._writer = csv.DictWriter(
            self._trajectory_fp,
            fieldnames=[
                "t",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "roll",
                "pitch",
                "yaw",
                "roll_cmd",
                "pitch_cmd",
                "yaw_cmd",
                "throttle_cmd",
                "distance_to_target_m",
            ],
        )
        self._writer.writeheader()

        self._observations_fp = (self.root / "observations.csv").open("w", newline="")
        self._observations_writer = csv.DictWriter(
            self._observations_fp,
            fieldnames=["frame_index", "t", "rgb_file", "depth_file", "metadata_json"],
        )
        self._observations_writer.writeheader()

        self._events: list[dict[str, Any]] = []
        self._frame_index = 0
        self._closed = False

        (self.root / "task.json").write_text(
            json.dumps(
                {
                    "episode_id": task.episode_id,
                    "start_position": task.start_position.tolist(),
                    "start_yaw": task.start_yaw,
                    "target": {
                        "target_id": task.target.target_id,
                        "position": task.target.position.tolist(),
                        "instruction": task.target.instruction,
                        "metadata": task.target.metadata,
                    },
                    "success_radius_m": task.success_radius_m,
                    "max_duration_s": task.max_duration_s,
                },
                indent=2,
                default=_json_default,
            )
        )

    def record(self, state: FlightState, command: ControlCommand, task: NavigationTask) -> None:
        self._writer.writerow(
            {
                "t": state.t,
                "x": state.position[0],
                "y": state.position[1],
                "z": state.position[2],
                "vx": state.velocity[0],
                "vy": state.velocity[1],
                "vz": state.velocity[2],
                "roll": state.rpy[0],
                "pitch": state.rpy[1],
                "yaw": state.rpy[2],
                "roll_cmd": command.roll,
                "pitch_cmd": command.pitch,
                "yaw_cmd": command.yaw,
                "throttle_cmd": command.throttle,
                "distance_to_target_m": task.distance_to_target(state.position),
            }
        )

    def record_observation(self, t: float, observation: Observation) -> None:
        """Store RGB/depth as lossless NumPy arrays plus one synchronization row."""

        index = self._frame_index
        self._frame_index += 1
        rgb_file = ""
        depth_file = ""

        if observation.rgb is not None:
            rgb_name = f"frame_{index:06d}.npy"
            np.save(self.rgb_dir / rgb_name, observation.rgb)
            rgb_file = f"rgb/{rgb_name}"

        if observation.depth is not None:
            depth_name = f"frame_{index:06d}.npy"
            np.save(self.depth_dir / depth_name, observation.depth)
            depth_file = f"depth/{depth_name}"

        metadata = observation.metadata or {}
        self._observations_writer.writerow(
            {
                "frame_index": index,
                "t": float(t),
                "rgb_file": rgb_file,
                "depth_file": depth_file,
                "metadata_json": json.dumps(
                    metadata,
                    separators=(",", ":"),
                    default=_json_default,
                ),
            }
        )

    def event(self, event_type: str, t: float, **payload: Any) -> None:
        self._events.append({"type": event_type, "t": float(t), **payload})

    def close(self) -> None:
        if self._closed:
            return
        self._trajectory_fp.close()
        self._observations_fp.close()
        (self.root / "events.json").write_text(
            json.dumps(self._events, indent=2, default=_json_default)
        )
        self._closed = True

    def __enter__(self) -> EpisodeRecorder:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
