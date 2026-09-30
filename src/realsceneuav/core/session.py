from __future__ import annotations

import time
from dataclasses import dataclass

from realsceneuav.controllers.base import Controller
from realsceneuav.controllers.events import ControllerEventType
from realsceneuav.controllers.interactive import InteractiveController
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
    termination_reason: str = "unknown"


class FlightSession:
    """Join human input, vehicle dynamics, scene observations and recording."""

    def __init__(
        self,
        scene: SceneAdapter,
        task: NavigationTask,
        dynamics: DynamicsBackend,
        controller: Controller,
        control_hz: float = 50.0,
        camera_hz: float = 10.0,
        realtime: bool = False,
    ) -> None:
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")
        if camera_hz <= 0:
            raise ValueError("camera_hz must be positive")
        if camera_hz > control_hz:
            raise ValueError("camera_hz cannot exceed control_hz in the current scheduler")

        self.scene = scene
        self.task = task
        self.dynamics = dynamics
        self.controller = controller
        self.control_hz = float(control_hz)
        self.camera_hz = float(camera_hz)
        self.realtime = bool(realtime)

    def _record_observation(self, recorder: EpisodeRecorder, state: FlightState) -> bool:
        try:
            observation = self.scene.observation(state.position, state.rpy)
        except NotImplementedError:
            return False
        recorder.record_observation(state.t, observation)
        return True

    def run(self, recorder: EpisodeRecorder) -> SessionResult:
        dt = 1.0 / self.control_hz
        camera_period = 1.0 / self.camera_hz

        initial = FlightState.zero()
        initial.position = self.task.start_position.copy()
        initial.rpy[2] = self.task.start_yaw
        state = self.dynamics.reset(initial)

        recorder.metadata(
            scene_id=self.scene.scene_id(),
            scene_adapter=type(self.scene).__name__,
            scene=self.scene.provenance(),
            control_hz=self.control_hz,
            camera_hz=self.camera_hz,
            realtime=self.realtime,
            controller=self.controller.provenance(),
            dynamics=self.dynamics.provenance(),
        )
        recorder.event("episode_start", state.t)
        observation_available = self._record_observation(recorder, state)
        if not observation_available:
            recorder.event("observation_unavailable", state.t)
        next_camera_t = state.t + camera_period

        paused = False
        next_wall_tick = time.monotonic()

        while state.t < self.task.max_duration_s:
            command = self.controller.poll()

            if isinstance(self.controller, InteractiveController):
                events = self.controller.consume_events()
                for event in events:
                    if event.type == ControllerEventType.PAUSE_TOGGLE:
                        paused = not paused
                        recorder.event("pause" if paused else "resume", state.t)
                    elif event.type == ControllerEventType.MARK_TARGET:
                        recorder.event(
                            "target_marker",
                            state.t,
                            position=state.position.tolist(),
                            distance_to_target_m=self.task.distance_to_target(state.position),
                        )
                    elif event.type == ControllerEventType.RESET:
                        reset_state = initial.copy()
                        reset_state.t = state.t
                        state = self.dynamics.reset(reset_state)
                        next_camera_t = state.t
                        recorder.event("reset", state.t)
                    elif event.type == ControllerEventType.STOP:
                        distance = self.task.distance_to_target(state.position)
                        success = self.task.success(state.position)
                        recorder.event(
                            "user_stop",
                            state.t,
                            success=success,
                            distance_to_target_m=distance,
                        )
                        return SessionResult(success, state.t, distance, "user_stop")

            if not paused:
                state = self.dynamics.step(command, dt)
                recorder.record(state, command, self.task)

                if observation_available and state.t + 1e-12 >= next_camera_t:
                    self._record_observation(recorder, state)
                    while next_camera_t <= state.t + 1e-12:
                        next_camera_t += camera_period

                if self.task.success(state.position):
                    recorder.event("success", state.t)
                    return SessionResult(
                        True,
                        state.t,
                        self.task.distance_to_target(state.position),
                        "success",
                    )

                ground = self.scene.ground_height(
                    float(state.position[0]), float(state.position[1])
                )
                if state.position[2] < ground:
                    recorder.event("ground_collision", state.t)
                    return SessionResult(
                        False,
                        state.t,
                        self.task.distance_to_target(state.position),
                        "ground_collision",
                    )

            if self.realtime:
                next_wall_tick += dt
                remaining = next_wall_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -5.0 * dt:
                    # Do not accumulate unbounded lag if rendering or the OS stalls.
                    next_wall_tick = time.monotonic()

        recorder.event("timeout", state.t)
        return SessionResult(
            False,
            state.t,
            self.task.distance_to_target(state.position),
            "timeout",
        )
