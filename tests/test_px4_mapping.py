import numpy as np

from realsceneuav.core.types import ControlCommand
from realsceneuav.dynamics.px4_mavlink import (
    command_to_px4_manual_control,
    px4_attitude_to_world_rpy,
    px4_ned_to_world_state,
)


def test_manual_control_mapping_matches_px4_ranges():
    command = ControlCommand(
        roll=1.0,
        pitch=-1.0,
        yaw=0.25,
        throttle=0.75,
    )
    x, y, z, r = command_to_px4_manual_control(command)

    assert x == -1000
    assert y == 1000
    assert z == 750
    assert r == 250


def test_px4_yaw_is_converted_from_north_clockwise_to_east_ccw():
    north_facing = px4_attitude_to_world_rpy(np.array([0.0, 0.0, 0.0]))
    east_facing = px4_attitude_to_world_rpy(np.array([0.0, 0.0, np.pi / 2]))

    assert np.isclose(north_facing[2], np.pi / 2)
    assert np.isclose(east_facing[2], 0.0, atol=1e-7)


def test_ned_position_velocity_are_mapped_to_world_enu():
    state = px4_ned_to_world_state(
        ned_position=np.array([11.0, 22.0, -8.0]),
        ned_velocity=np.array([1.0, 2.0, -3.0]),
        px4_rpy=np.array([0.0, 0.0, 0.0]),
        body_rates_frd=np.array([0.1, 0.2, 0.3]),
        anchor_ned=np.array([10.0, 20.0, -5.0]),
        world_origin=np.array([100.0, 200.0, 50.0]),
        frame_yaw_offset=0.0,
        t=1.5,
    )

    assert np.allclose(state.position, [102.0, 201.0, 53.0])
    assert np.allclose(state.velocity, [2.0, 1.0, 3.0])
    assert np.allclose(state.angular_velocity, [0.1, -0.2, -0.3])
    assert np.isclose(state.rpy[2], np.pi / 2)
    assert state.t == 1.5


def test_frame_yaw_offset_rotates_px4_local_motion_into_citynav_frame():
    state = px4_ned_to_world_state(
        ned_position=np.array([1.0, 0.0, 0.0]),
        ned_velocity=np.array([1.0, 0.0, 0.0]),
        px4_rpy=np.array([0.0, 0.0, 0.0]),
        body_rates_frd=np.zeros(3),
        anchor_ned=np.zeros(3),
        world_origin=np.zeros(3),
        frame_yaw_offset=-np.pi / 2,
        t=0.0,
    )

    assert np.allclose(state.position, [1.0, 0.0, 0.0], atol=1e-7)
    assert np.allclose(state.velocity, [1.0, 0.0, 0.0], atol=1e-7)
    assert np.isclose(state.rpy[2], 0.0, atol=1e-7)
