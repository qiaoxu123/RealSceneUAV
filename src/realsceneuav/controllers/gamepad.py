from __future__ import annotations

from dataclasses import dataclass

from realsceneuav.controllers.base import Controller
from realsceneuav.core.types import ControlCommand


@dataclass(slots=True)
class GamepadMapping:
    """SDL axis mapping; override in YAML for different controllers."""

    roll_axis: int = 2
    pitch_axis: int = 3
    yaw_axis: int = 0
    throttle_axis: int = 1
    invert_roll: bool = False
    invert_pitch: bool = True
    invert_yaw: bool = False
    invert_throttle: bool = True
    deadzone: float = 0.06


class PygameGamepadController(Controller):
    """Generic SDL controller for Switch Pro Controller, Joy-Con pairs, etc."""

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
        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()

    def _axis(self, index: int, invert: bool) -> float:
        value = float(self.joystick.get_axis(index))
        if invert:
            value = -value
        if abs(value) < self.mapping.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def poll(self) -> ControlCommand:
        self._pygame.event.pump()
        m = self.mapping
        throttle_raw = self._axis(m.throttle_axis, m.invert_throttle)
        throttle = (throttle_raw + 1.0) / 2.0
        return ControlCommand(
            roll=self._axis(m.roll_axis, m.invert_roll),
            pitch=self._axis(m.pitch_axis, m.invert_pitch),
            yaw=self._axis(m.yaw_axis, m.invert_yaw),
            throttle=throttle,
        ).clipped()
