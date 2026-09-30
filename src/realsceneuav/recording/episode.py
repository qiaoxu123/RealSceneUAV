from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.tasks.navigation import NavigationTask


class EpisodeRecorder:
    """Write task metadata and synchronized human-control/vehicle-state samples."""

    def __init__(self, output_root: str | Path, task: NavigationTask) -> None:
        self.root = Path(output_root) / task.episode_id
        self.root.mkdir(parents=True, exist_ok=False)
        self._trajectory_fp = (self.root / "trajectory.csv").open("w", newline="")
        self._writer = csv.DictWriter(
            self._trajectory_fp,
            fieldnames=[
                "t", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw",
                "roll_cmd", "pitch_cmd", "yaw_cmd", "throttle_cmd",
                "distance_to_target_m",
            ],
        )
        self._writer.writeheader()
        self._events: list[dict[str, Any]] = []
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

    def event(self, event_type: str, t: float, **payload: Any) -> None:
        self._events.append({"type": event_type, "t": t, **payload})

    def close(self) -> None:
        self._trajectory_fp.close()
        (self.root / "events.json").write_text(json.dumps(self._events, indent=2))

    def __enter__(self) -> "EpisodeRecorder":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
