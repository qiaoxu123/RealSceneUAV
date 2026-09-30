from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np


@dataclass(slots=True)
class ControlCommand:
    """Normalized manual control command.

    roll, pitch, yaw are in [-1, 1]. Throttle is in [0, 1].
    """

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.5

    def clipped(self) -> "ControlCommand":
        return ControlCommand(
            roll=float(np.clip(self.roll, -1.0, 1.0)),
            pitch=float(np.clip(self.pitch, -1.0, 1.0)),
            yaw=float(np.clip(self.yaw, -1.0, 1.0)),
            throttle=float(np.clip(self.throttle, 0.0, 1.0)),
        )

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(slots=True)
class FlightState:
    """Minimal 6-DoF state used across all backends."""

    t: float
    position: np.ndarray
    velocity: np.ndarray
    rpy: np.ndarray
    angular_velocity: np.ndarray

    @classmethod
    def zero(cls) -> "FlightState":
        z = np.zeros(3, dtype=np.float64)
        return cls(0.0, z.copy(), z.copy(), z.copy(), z.copy())

    def copy(self) -> "FlightState":
        return FlightState(
            t=float(self.t),
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            rpy=self.rpy.copy(),
            angular_velocity=self.angular_velocity.copy(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "t": float(self.t),
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "rpy": self.rpy.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
        }
