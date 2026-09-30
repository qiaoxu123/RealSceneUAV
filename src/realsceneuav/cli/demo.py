from __future__ import annotations

import argparse
from dataclasses import dataclass

from realsceneuav.controllers.base import Controller
from realsceneuav.core.session import FlightSession
from realsceneuav.core.types import ControlCommand
from realsceneuav.dynamics.reference import ReferenceQuadrotorDynamics
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.mock import MockRealScene
from realsceneuav.tasks.sampler import sample_task


@dataclass
class HoverController(Controller):
    throttle: float = 1.0 / 2.2

    def poll(self) -> ControlCommand:
        return ControlCommand(throttle=self.throttle)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="outputs")
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()

    scene = MockRealScene()
    task = sample_task(scene, seed=0)
    task.max_duration_s = args.seconds

    with EpisodeRecorder(args.output, task) as recorder:
        result = FlightSession(
            scene,
            task,
            ReferenceQuadrotorDynamics(),
            HoverController(),
        ).run(recorder)
    print(result)


if __name__ == "__main__":
    main()
