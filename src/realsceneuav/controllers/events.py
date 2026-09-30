from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ControllerEventType(str, Enum):
    """Discrete operator events kept separate from continuous stick commands."""

    PAUSE_TOGGLE = "pause_toggle"
    STOP = "stop"
    MARK_TARGET = "mark_target"
    RESET = "reset"


@dataclass(frozen=True, slots=True)
class ControllerEvent:
    type: ControllerEventType
