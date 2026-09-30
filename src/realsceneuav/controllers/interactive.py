from __future__ import annotations

from abc import abstractmethod

from realsceneuav.controllers.base import Controller
from realsceneuav.controllers.events import ControllerEvent


class InteractiveController(Controller):
    """Controller that also emits edge-triggered operator events."""

    @abstractmethod
    def consume_events(self) -> list[ControllerEvent]:
        raise NotImplementedError
