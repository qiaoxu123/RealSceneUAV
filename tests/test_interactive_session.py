import csv
import json

from realsceneuav.controllers.events import ControllerEvent, ControllerEventType
from realsceneuav.controllers.scripted import ScriptedController
from realsceneuav.core.session import FlightSession
from realsceneuav.dynamics.reference import ReferenceQuadrotorDynamics
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.mock import MockRealScene
from realsceneuav.tasks.sampler import sample_task


def test_interactive_session_records_events_observations_and_metadata(tmp_path):
    scene = MockRealScene()
    task = sample_task(scene, seed=9)
    task.max_duration_s = 2.0

    controller = ScriptedController(
        events_by_poll={
            1: [ControllerEvent(ControllerEventType.MARK_TARGET)],
            2: [ControllerEvent(ControllerEventType.PAUSE_TOGGLE)],
            3: [ControllerEvent(ControllerEventType.PAUSE_TOGGLE)],
            4: [ControllerEvent(ControllerEventType.RESET)],
            7: [ControllerEvent(ControllerEventType.STOP)],
        }
    )

    with EpisodeRecorder(tmp_path, task) as recorder:
        result = FlightSession(
            scene=scene,
            task=task,
            dynamics=ReferenceQuadrotorDynamics(),
            controller=controller,
            control_hz=20.0,
            camera_hz=10.0,
            realtime=False,
        ).run(recorder)

    assert result.termination_reason == "user_stop"

    root = tmp_path / task.episode_id
    events = json.loads((root / "events.json").read_text())
    event_types = [event["type"] for event in events]
    assert event_types == [
        "episode_start",
        "target_marker",
        "pause",
        "resume",
        "reset",
        "user_stop",
    ]

    with (root / "trajectory.csv").open() as fp:
        rows = list(csv.DictReader(fp))
    times = [float(row["t"]) for row in rows]
    assert times == sorted(times)

    with (root / "observations.csv").open() as fp:
        observations = list(csv.DictReader(fp))
    assert len(observations) >= 2
    for observation in observations:
        if observation["rgb_file"]:
            assert (root / observation["rgb_file"]).exists()
        if observation["depth_file"]:
            assert (root / observation["depth_file"]).exists()

    metadata = json.loads((root / "metadata.json").read_text())
    assert metadata["scene_id"] == "mock-real-scene"
    assert metadata["control_hz"] == 20.0
    assert metadata["camera_hz"] == 10.0
    assert metadata["controller"]["backend"] == "ScriptedController"
    assert metadata["dynamics"]["backend"] == "ReferenceQuadrotorDynamics"
    assert metadata["dynamics"]["validated_real_aircraft_model"] is False


def test_user_stop_reports_success_when_already_at_target(tmp_path):
    scene = MockRealScene()
    task = sample_task(scene, seed=2)
    task.start_position = task.target.position.copy()
    task.max_duration_s = 1.0

    controller = ScriptedController(
        events_by_poll={0: [ControllerEvent(ControllerEventType.STOP)]}
    )

    with EpisodeRecorder(tmp_path, task) as recorder:
        result = FlightSession(
            scene=scene,
            task=task,
            dynamics=ReferenceQuadrotorDynamics(),
            controller=controller,
            control_hz=20.0,
            camera_hz=10.0,
            realtime=False,
        ).run(recorder)

    assert result.success is True
    assert result.termination_reason == "user_stop"
    assert result.final_distance_m == 0.0
