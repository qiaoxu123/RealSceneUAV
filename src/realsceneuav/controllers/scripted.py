from __future__ import annotations

from dataclasses import dataclass, field

from realsceneuav.controllers.events import ControllerEvent
from realsceneuav.controllers.interactive import InteractiveController
from realsceneuav.core.types import ControlCommand


@dataclass
class ScriptedController(InteractiveController):
    """Deterministic controller for demos, CI and replay-oriented tests."""

    command: ControlCommand = field(default_factory=lambda: ControlCommand(throttle=1.0 / 2.2))
    events_by_poll: dict[int, list[ControllerEvent]] = field(default_factory=dict)
    _poll_index: int = 0
    _pending: list[ControllerEvent] = field(default_factory=list)

    def poll(self) -> ControlCommand:
        self._pending = list(self.events_by_poll.get(self._poll_index, []))
        self._poll_index += 1
        return self.command

    def consume_events(self) -> list[ControllerEvent]:
        events = self._pending
        self._pending = []
        return events
