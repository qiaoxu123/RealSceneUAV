from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import yaml

from realsceneuav.controllers.events import ControllerEvent, ControllerEventType
from realsceneuav.controllers.interactive import InteractiveController
from realsceneuav.core.types import ControlCommand


@dataclass(slots=True)
class GamepadMapping:
    """SDL axis/button mapping; override after running controller-check."""

    roll_axis: int = 2
    pitch_axis: int = 3
    yaw_axis: int = 0
    throttle_axis: int = 1
    invert_roll: bool = False
    invert_pitch: bool = True
    invert_yaw: bool = False
    invert_throttle: bool = True
    deadzone: float = 0.06

    # SDL joystick button indices are driver/platform dependent.
    mark_target_button: int = 0
    stop_button: int = 1
    reset_button: int = 3
    pause_button: int = 6


def load_gamepad_config(path: str | Path) -> tuple[GamepadMapping, int]:
    """Load a reproducible SDL mapping and joystick index from YAML."""

    config_path = Path(path)
    raw = yaml.safe_load(config_path.read_text()) or {}
    controller = raw.get("controller", {})
    if controller.get("type", "pygame") != "pygame":
        raise ValueError("Only controller.type=pygame is supported by this backend")

    mapping = GamepadMapping(**(controller.get("mapping") or {}))
    joystick_index = int(controller.get("joystick_index", 0))
    return mapping, joystick_index


class PygameGamepadController(InteractiveController):
    """Generic SDL controller for Switch Pro Controller, Joy-Con pairs, etc.

    Continuous stick values and discrete operator events are deliberately separated.
    Buttons use rising-edge detection so holding a button never floods the recorder.
    """

    def __init__(self, mapping: GamepadMapping | None = None, joystick_index: int = 0) -> None:
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("Install realsceneuav[controller] to use a gamepad") from exc

        self._pygame = pygame
        self.mapping = mapping or GamepadMapping()
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= joystick_index:
            raise RuntimeError("No compatible gamepad detected")
        self.joystick_index = int(joystick_index)
        self.joystick = pygame.joystick.Joystick(self.joystick_index)
        self.joystick.init()

        self._pending_events: list[ControllerEvent] = []
        self._previous_buttons = [
            int(self.joystick.get_button(i)) for i in range(self.joystick.get_numbuttons())
        ]

    def _axis(self, index: int, invert: bool) -> float:
        value = float(self.joystick.get_axis(index))
        if invert:
            value = -value
        if abs(value) < self.mapping.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def _button_pressed(self, index: int, current: list[int]) -> bool:
        if index < 0 or index >= len(current):
            return False
        return current[index] == 1 and self._previous_buttons[index] == 0

    def _poll_button_events(self) -> None:
        current = [
            int(self.joystick.get_button(i)) for i in range(self.joystick.get_numbuttons())
        ]
        mapping = (
            (self.mapping.pause_button, ControllerEventType.PAUSE_TOGGLE),
            (self.mapping.stop_button, ControllerEventType.STOP),
            (self.mapping.mark_target_button, ControllerEventType.MARK_TARGET),
            (self.mapping.reset_button, ControllerEventType.RESET),
        )
        for button, event_type in mapping:
            if self._button_pressed(button, current):
                self._pending_events.append(ControllerEvent(event_type))
        self._previous_buttons = current

    def poll(self) -> ControlCommand:
        self._pygame.event.pump()
        self._poll_button_events()
        m = self.mapping
        throttle_raw = self._axis(m.throttle_axis, m.invert_throttle)
        throttle = (throttle_raw + 1.0) / 2.0
        return ControlCommand(
            roll=self._axis(m.roll_axis, m.invert_roll),
            pitch=self._axis(m.pitch_axis, m.invert_pitch),
            yaw=self._axis(m.yaw_axis, m.invert_yaw),
            throttle=throttle,
        ).clipped()

    def consume_events(self) -> list[ControllerEvent]:
        events = self._pending_events
        self._pending_events = []
        return events


    def provenance(self) -> dict[str, object]:
        return {
            "backend": type(self).__name__,
            "joystick_index": self.joystick_index,
            "device_name": self.joystick.get_name(),
            "num_axes": self.joystick.get_numaxes(),
            "num_buttons": self.joystick.get_numbuttons(),
            "mapping": asdict(self.mapping),
        }
