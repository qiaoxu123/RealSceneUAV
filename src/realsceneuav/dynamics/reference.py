from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.dynamics.base import DynamicsBackend


@dataclass(slots=True)
class ReferenceDynamicsConfig:
    mass_kg: float = 1.5
    gravity_mps2: float = 9.81
    max_tilt_deg: float = 35.0
    max_yaw_rate_deg_s: float = 120.0
    max_thrust_to_weight: float = 2.2
    attitude_time_constant_s: float = 0.18
    linear_drag: float = 0.12


class ReferenceQuadrotorDynamics(DynamicsBackend):
    """Small, deterministic rigid-body reference model.

    This is intentionally not presented as a validated aircraft model. It exists so
    the whole platform can run without PX4. Production experiments should swap this
    backend for PX4 SITL/HIL, RotorPy, or another validated dynamics implementation.
    """

    def __init__(self, config: ReferenceDynamicsConfig | None = None) -> None:
        self.config = config or ReferenceDynamicsConfig()
        self._state = FlightState.zero()

    def reset(self, state: FlightState | None = None) -> FlightState:
        self._state = state.copy() if state is not None else FlightState.zero()
        return self._state.copy()

    def state(self) -> FlightState:
        return self._state.copy()

    @staticmethod
    def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )

    def step(self, command: ControlCommand, dt: float) -> FlightState:
        if dt <= 0:
            raise ValueError("dt must be positive")
        c = command.clipped()
        cfg = self.config

        max_tilt = np.deg2rad(cfg.max_tilt_deg)
        desired_roll = c.roll * max_tilt
        desired_pitch = c.pitch * max_tilt
        yaw_rate = c.yaw * np.deg2rad(cfg.max_yaw_rate_deg_s)

        alpha = 1.0 - np.exp(-dt / max(cfg.attitude_time_constant_s, 1e-3))
        new_rpy = self._state.rpy.copy()
        new_rpy[0] += alpha * (desired_roll - new_rpy[0])
        new_rpy[1] += alpha * (desired_pitch - new_rpy[1])
        new_rpy[2] += yaw_rate * dt

        thrust_n = c.throttle * cfg.max_thrust_to_weight * cfg.mass_kg * cfg.gravity_mps2
        body_z_world = self._rotation_matrix(*new_rpy)[:, 2]
        gravity = np.array([0.0, 0.0, cfg.gravity_mps2])
        acceleration = (thrust_n / cfg.mass_kg) * body_z_world - gravity
        acceleration -= cfg.linear_drag * self._state.velocity

        new_velocity = self._state.velocity + acceleration * dt
        new_position = self._state.position + new_velocity * dt

        self._state = FlightState(
            t=self._state.t + dt,
            position=new_position,
            velocity=new_velocity,
            rpy=new_rpy,
            angular_velocity=np.array([0.0, 0.0, yaw_rate], dtype=np.float64),
        )
        return self._state.copy()

    def provenance(self) -> dict[str, object]:
        return {
            "backend": type(self).__name__,
            "config": asdict(self.config),
            "validated_real_aircraft_model": False,
        }
