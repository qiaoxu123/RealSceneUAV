import numpy as np

from realsceneuav.core.types import ControlCommand
from realsceneuav.dynamics.reference import ReferenceQuadrotorDynamics


def test_hover_is_approximately_stationary():
    dyn = ReferenceQuadrotorDynamics()
    dyn.reset()
    hover_throttle = 1.0 / dyn.config.max_thrust_to_weight
    for _ in range(200):
        state = dyn.step(ControlCommand(throttle=hover_throttle), 0.01)
    assert abs(state.position[2]) < 0.05
    assert np.linalg.norm(state.velocity) < 0.05


def test_pitch_moves_vehicle_horizontally():
    dyn = ReferenceQuadrotorDynamics()
    dyn.reset()
    hover_throttle = 1.0 / dyn.config.max_thrust_to_weight
    for _ in range(200):
        state = dyn.step(ControlCommand(pitch=0.5, throttle=hover_throttle), 0.01)
    assert abs(state.position[0]) > 0.1
