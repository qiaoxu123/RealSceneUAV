from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from realsceneuav.core.types import ControlCommand, FlightState


class DynamicsBackend(ABC):
    """Interface for interchangeable vehicle dynamics backends."""

    @abstractmethod
    def reset(self, state: FlightState | None = None) -> FlightState:
        raise NotImplementedError

    @abstractmethod
    def step(self, command: ControlCommand, dt: float) -> FlightState:
        raise NotImplementedError

    @abstractmethod
    def state(self) -> FlightState:
        raise NotImplementedError

    def supports_state_reset(self) -> bool:
        """Whether reset(state) can physically move the simulated vehicle to state."""

        return True

    def supports_pause_freeze(self) -> bool:
        """Whether the backend can freeze its physical state while a session is paused."""

        return True

    def provenance(self) -> dict[str, Any]:
        """Serializable description of the vehicle backend used for an episode."""

        return {"backend": type(self).__name__}
