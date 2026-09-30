from __future__ import annotations

from dataclasses import dataclass

from realsceneuav.controllers.base import Controller
from realsceneuav.core.types import FlightState
from realsceneuav.dynamics.base import DynamicsBackend
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.base import SceneAdapter
from realsceneuav.tasks.navigation import NavigationTask


@dataclass(slots=True)
class SessionResult:
    success: bool
    duration_s: float
    final_distance_m: float


class FlightSession:
    """Central loop joining input, vehicle dynamics, scene and data recording."""

    def __init__(
        self,
        scene: SceneAdapter,
        task: NavigationTask,
        dynamics: DynamicsBackend,
        controller: Controller,
        control_hz: float = 50.0,
    ) -> None:
        self.scene = scene
        self.task = task
        self.dynamics = dynamics
        self.controller = controller
        self.control_hz = float(control_hz)

    def run(self, recorder: EpisodeRecorder) -> SessionResult:
        dt = 1.0 / self.control_hz
        initial = FlightState.zero()
        initial.position = self.task.start_position.copy()
        initial.rpy[2] = self.task.start_yaw
        state = self.dynamics.reset(initial)

        recorder.event("episode_start", state.t)
        while state.t < self.task.max_duration_s:
            command = self.controller.poll()
            state = self.dynamics.step(command, dt)
            recorder.record(state, command, self.task)

            if self.task.success(state.position):
                recorder.event("success", state.t)
                return SessionResult(True, state.t, self.task.distance_to_target(state.position))

            ground = self.scene.ground_height(float(state.position[0]), float(state.position[1]))
            if state.position[2] < ground:
                recorder.event("ground_collision", state.t)
                return SessionResult(False, state.t, self.task.distance_to_target(state.position))

        recorder.event("timeout", state.t)
        return SessionResult(False, state.t, self.task.distance_to_target(state.position))
