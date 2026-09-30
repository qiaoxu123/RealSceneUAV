from __future__ import annotations

import argparse

from realsceneuav.controllers.gamepad import PygameGamepadController
from realsceneuav.controllers.scripted import ScriptedController
from realsceneuav.core.session import FlightSession
from realsceneuav.dynamics.reference import ReferenceQuadrotorDynamics
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.mock import MockRealScene
from realsceneuav.tasks.sampler import sample_task


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect one interactive UAV navigation episode.")
    parser.add_argument("--output", default="outputs")
    parser.add_argument("--controller", choices=["gamepad", "scripted"], default="gamepad")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--joystick-index", type=int, default=0)
    parser.add_argument("--no-realtime", action="store_true")
    args = parser.parse_args()

    scene = MockRealScene()
    task = sample_task(scene, seed=args.seed)
    task.max_duration_s = args.duration

    if args.controller == "gamepad":
        controller = PygameGamepadController(joystick_index=args.joystick_index)
    else:
        controller = ScriptedController()

    with EpisodeRecorder(args.output, task) as recorder:
        result = FlightSession(
            scene=scene,
            task=task,
            dynamics=ReferenceQuadrotorDynamics(),
            controller=controller,
            control_hz=args.control_hz,
            camera_hz=args.camera_hz,
            realtime=not args.no_realtime,
        ).run(recorder)

    print(result)


if __name__ == "__main__":
    main()
