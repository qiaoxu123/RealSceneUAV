from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from realsceneuav.controllers.base import Controller
from realsceneuav.controllers.events import ControllerEventType
from realsceneuav.controllers.interactive import InteractiveController
from realsceneuav.core.observer import SessionObserver
from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.dynamics.base import DynamicsBackend
from realsceneuav.recording.episode import EpisodeRecorder
from realsceneuav.scenes.base import Observation, SceneAdapter
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
        observers: list[SessionObserver] | None = None,
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
        self.observers = list(observers or [])

    def _notify_start(self, state: FlightState) -> None:
        for observer in self.observers:
            observer.on_start(self.scene, self.task, state.copy())

    def _notify_state(
        self,
        state: FlightState,
        command: ControlCommand,
        paused: bool,
    ) -> None:
        for observer in self.observers:
            observer.on_state(state.copy(), command, paused)

    def _emit_event(
        self,
        recorder: EpisodeRecorder,
        event_type: str,
        state: FlightState,
        **payload: Any,
    ) -> None:
        recorder.event(event_type, state.t, **payload)
        for observer in self.observers:
            observer.on_event(event_type, state.copy(), dict(payload))

    def _record_observation(
        self,
        recorder: EpisodeRecorder,
        state: FlightState,
    ) -> Observation | None:
        try:
            observation = self.scene.observation(state.position, state.rpy)
        except NotImplementedError:
            return None

        recorder.record_observation(state.t, observation)
        for observer in self.observers:
            observer.on_observation(state.copy(), observation)
        return observation

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
        self._notify_start(state)
        self._emit_event(recorder, "episode_start", state)

        observation_available = self._record_observation(recorder, state) is not None
        if not observation_available:
            self._emit_event(recorder, "observation_unavailable", state)
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
                        self._emit_event(
                            recorder,
                            "pause" if paused else "resume",
                            state,
                        )
                    elif event.type == ControllerEventType.MARK_TARGET:
                        self._emit_event(
                            recorder,
                            "target_marker",
                            state,
                            position=state.position.tolist(),
                            distance_to_target_m=self.task.distance_to_target(state.position),
                        )
                    elif event.type == ControllerEventType.RESET:
                        if self.dynamics.supports_state_reset():
                            reset_state = initial.copy()
                            reset_state.t = state.t
                            state = self.dynamics.reset(reset_state)
                            next_camera_t = state.t
                            self._emit_event(recorder, "reset", state)
                            self._notify_state(state, command, paused)
                        else:
                            self._emit_event(
                                recorder,
                                "reset_unsupported",
                                state,
                                backend=type(self.dynamics).__name__,
                            )
                    elif event.type == ControllerEventType.STOP:
                        distance = self.task.distance_to_target(state.position)
                        success = self.task.success(state.position)
                        self._emit_event(
                            recorder,
                            "user_stop",
                            state,
                            success=success,
                            distance_to_target_m=distance,
                        )
                        return SessionResult(success, state.t, distance, "user_stop")

            if not paused:
                state = self.dynamics.step(command, dt)
                recorder.record(state, command, self.task)
                self._notify_state(state, command, paused=False)

                if observation_available and state.t + 1e-12 >= next_camera_t:
                    self._record_observation(recorder, state)
                    while next_camera_t <= state.t + 1e-12:
                        next_camera_t += camera_period

                if self.task.success(state.position):
                    self._emit_event(recorder, "success", state)
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
                    self._emit_event(recorder, "ground_collision", state)
                    return SessionResult(
                        False,
                        state.t,
                        self.task.distance_to_target(state.position),
                        "ground_collision",
                    )
            else:
                self._notify_state(state, command, paused=True)

            if self.realtime:
                next_wall_tick += dt
                remaining = next_wall_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -5.0 * dt:
                    next_wall_tick = time.monotonic()

        self._emit_event(recorder, "timeout", state)
        return SessionResult(
            False,
            state.t,
            self.task.distance_to_target(state.position),
            "timeout",
        )

    def close_observers(self) -> None:
        for observer in self.observers:
            observer.close()
