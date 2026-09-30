from realsceneuav.controllers.events import ControllerEvent, ControllerEventType
from realsceneuav.controllers.scripted import ScriptedController
from realsceneuav.core.observer import SessionObserver
from realsceneuav.core.session import FlightSession
from realsceneuav.dynamics.reference import ReferenceQuadrotorDynamics
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.mock import MockRealScene
from realsceneuav.tasks.sampler import sample_task


class CaptureObserver(SessionObserver):
    def __init__(self):
        self.starts = 0
        self.state_times = []
        self.observation_times = []
        self.events = []

    def on_start(self, scene, task, state):
        del scene, task
        self.starts += 1
        self.state_times.append(state.t)

    def on_state(self, state, command, paused):
        del command, paused
        self.state_times.append(state.t)

    def on_observation(self, state, observation):
        del observation
        self.observation_times.append(state.t)

    def on_event(self, event_type, state, payload):
        del state, payload
        self.events.append(event_type)


def test_session_observer_receives_live_hooks(tmp_path):
    scene = MockRealScene()
    task = sample_task(scene, seed=4)
    task.max_duration_s = 1.0

    controller = ScriptedController(
        events_by_poll={
            3: [ControllerEvent(ControllerEventType.STOP)],
        }
    )
    observer = CaptureObserver()

    with EpisodeRecorder(tmp_path, task) as recorder:
        result = FlightSession(
            scene=scene,
            task=task,
            dynamics=ReferenceQuadrotorDynamics(),
            controller=controller,
            control_hz=20.0,
            camera_hz=10.0,
            realtime=False,
            observers=[observer],
        ).run(recorder)

    assert result.termination_reason == "user_stop"
    assert observer.starts == 1
    assert len(observer.state_times) >= 2
    assert len(observer.observation_times) >= 1
    assert observer.events[0] == "episode_start"
    assert observer.events[-1] == "user_stop"
