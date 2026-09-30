from __future__ import annotations

import argparse
from datetime import datetime, timezone

from realsceneuav.controllers.gamepad import PygameGamepadController, load_gamepad_config
from realsceneuav.controllers.scripted import ScriptedController
from realsceneuav.core.session import FlightSession
from realsceneuav.dynamics.px4_mavlink import Px4MavlinkBackend, Px4MavlinkConfig
from realsceneuav.dynamics.reference import ReferenceQuadrotorDynamics
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.citynav import CityNavTrajectoryAdapter
from realsceneuav.scenes.citynav_raster import CityNavRasterScene, RasterShape
from realsceneuav.viewers.citynav_live import CityNavLiveViewer


def _default_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Collect a human-controlled episode in a real CityNav/SensatUrban scene."
    )
    parser.add_argument("--trajectory-json", required=True)
    parser.add_argument("--episode-index", type=int, required=True)
    parser.add_argument("--rgb-png", required=True)
    parser.add_argument("--height-tif", required=True)
    parser.add_argument("--ground-level", type=float, default=None)
    parser.add_argument("--output", default="outputs")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--controller", choices=["gamepad", "scripted"], default="gamepad")
    parser.add_argument("--controller-config", default="configs/switch_pro.yaml")
    parser.add_argument("--joystick-index", type=int, default=None)
    parser.add_argument("--dynamics", choices=["reference", "px4"], default="reference")
    parser.add_argument("--px4-connection", default="udpin:0.0.0.0:14540")
    parser.add_argument("--px4-telemetry-hz", type=float, default=50.0)
    parser.add_argument("--px4-mode", default="POSCTL")
    parser.add_argument("--px4-arm", action="store_true")
    parser.add_argument("--px4-disarm-on-exit", action="store_true")
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--success-radius", type=float, default=5.0)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--camera-width", type=int, default=384)
    parser.add_argument("--camera-height", type=int, default=384)
    parser.add_argument("--panel-size", type=int, default=420)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    args = parser.parse_args()

    metadata = CityNavTrajectoryAdapter(args.trajectory_json)
    index = args.episode_index
    map_name = metadata.map_name(index)

    task = metadata.navigation_task(
        index,
        success_radius_m=args.success_radius,
        max_duration_s=args.duration,
    )
    run_id = args.run_id or _default_run_id()
    task.episode_id = f"{task.episode_id}-{run_id}"

    targets = [
        metadata.target_for_record(i)
        for i in metadata.source_indices_for_map(map_name)
        if metadata.source_record(i).get("target_positions")
    ]
    scene = CityNavRasterScene(
        map_name=map_name,
        rgb_png=args.rgb_png,
        height_tif=args.height_tif,
        targets=targets,
        ground_level_m=args.ground_level,
        output_shape=RasterShape(
            height=args.camera_height,
            width=args.camera_width,
        ),
    )

    if args.controller == "gamepad":
        mapping, configured_index = load_gamepad_config(args.controller_config)
        joystick_index = configured_index if args.joystick_index is None else args.joystick_index
        controller = PygameGamepadController(
            mapping=mapping,
            joystick_index=joystick_index,
        )
    else:
        controller = ScriptedController()

    if args.dynamics == "px4":
        if args.no_realtime:
            raise SystemExit("--no-realtime is not valid with --dynamics px4")
        if args.control_hz < 10.0:
            raise SystemExit("PX4 manual control should use --control-hz >= 10")

        dynamics = Px4MavlinkBackend(
            Px4MavlinkConfig(
                connection_url=args.px4_connection,
                telemetry_hz=args.px4_telemetry_hz,
            )
        )
        dynamics.prime_manual_control()
        if args.px4_mode:
            dynamics.set_mode(args.px4_mode)
        if args.px4_arm:
            dynamics.arm()
    else:
        dynamics = ReferenceQuadrotorDynamics()

    observers = []
    if not args.no_viewer:
        observers.append(
            CityNavLiveViewer(
                scene=scene,
                task=task,
                panel_size=args.panel_size,
            )
        )

    session = FlightSession(
        scene=scene,
        task=task,
        dynamics=dynamics,
        controller=controller,
        control_hz=args.control_hz,
        camera_hz=args.camera_hz,
        realtime=not args.no_realtime,
        observers=observers,
    )

    try:
        with EpisodeRecorder(args.output, task) as recorder:
            recorder.metadata(
                citynav_source={
                    "trajectory_json": args.trajectory_json,
                    "source_episode_index": index,
                    "source_map_name": map_name,
                },
                collection_run_id=run_id,
            )
            result = session.run(recorder)
    finally:
        session.close_observers()
        if (
            args.dynamics == "px4"
            and args.px4_disarm_on_exit
            and isinstance(dynamics, Px4MavlinkBackend)
        ):
            dynamics.disarm()

    print(result)


if __name__ == "__main__":
    main()
