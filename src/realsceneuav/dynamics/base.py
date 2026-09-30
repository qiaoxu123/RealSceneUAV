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

    def provenance(self) -> dict[str, Any]:
        """Serializable description of the vehicle backend used for an episode."""

        return {"backend": type(self).__name__}
