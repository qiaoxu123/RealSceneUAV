from __future__ import annotations

from typing import Any

from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.scenes.base import Observation, SceneAdapter
from realsceneuav.tasks.navigation import NavigationTask


class SessionObserver:
    """Optional non-owning hook for live viewers, telemetry, or debugging."""

    def on_start(
        self,
        scene: SceneAdapter,
        task: NavigationTask,
        state: FlightState,
    ) -> None:
        pass

    def on_state(
        self,
        state: FlightState,
        command: ControlCommand,
        paused: bool,
    ) -> None:
        pass

    def on_observation(
        self,
        state: FlightState,
        observation: Observation,
    ) -> None:
        pass

    def on_event(
        self,
        event_type: str,
        state: FlightState,
        payload: dict[str, Any],
    ) -> None:
        pass

    def close(self) -> None:
        pass
