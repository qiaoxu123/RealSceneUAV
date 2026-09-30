from __future__ import annotations

import time
from dataclasses import asdict, dataclass

import numpy as np

from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.dynamics.base import DynamicsBackend


@dataclass(slots=True)
class Px4MavlinkConfig:
    """Connection and telemetry settings for a PX4 MAVLink backend."""

    connection_url: str = "udpin:0.0.0.0:14540"
    heartbeat_timeout_s: float = 10.0
    telemetry_timeout_s: float = 2.0
    telemetry_hz: float = 50.0
    source_system: int = 245
    source_component: int = 190


def command_to_px4_manual_control(command: ControlCommand) -> tuple[int, int, int, int]:
    """Map RealSceneUAV normalized controls to PX4 MANUAL_CONTROL axes.

    PX4 currently interprets x/y/r in [-1000, 1000] and keeps backwards-compatible
    throttle z in [0, 1000].
    """

    command = command.clipped()
    x = int(round(command.pitch * 1000.0))
    y = int(round(command.roll * 1000.0))
    z = int(round(command.throttle * 1000.0))
    r = int(round(command.yaw * 1000.0))
    return (
        int(np.clip(x, -1000, 1000)),
        int(np.clip(y, -1000, 1000)),
        int(np.clip(z, 0, 1000)),
        int(np.clip(r, -1000, 1000)),
    )


def _rotation_matrix_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
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


def _matrix_to_rpy_zyx(rotation: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    cp = np.cos(pitch)
    if abs(cp) > 1e-8:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def _rotation_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


_NED_TO_ENU = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=np.float64,
)

_FRD_FROM_FLU = np.diag([1.0, -1.0, -1.0]).astype(np.float64)


def px4_attitude_to_world_rpy(
    px4_rpy: np.ndarray,
    frame_yaw_offset: float = 0.0,
) -> np.ndarray:
    """Convert PX4 NED/FRD attitude to an ENU/FLU-style world RPY."""

    roll, pitch, yaw = np.asarray(px4_rpy, dtype=np.float64)
    rotation_ned_frd = _rotation_matrix_zyx(roll, pitch, yaw)
    rotation_enu_flu = _NED_TO_ENU @ rotation_ned_frd @ _FRD_FROM_FLU
    rotation_world_flu = _rotation_z(frame_yaw_offset) @ rotation_enu_flu
    return _matrix_to_rpy_zyx(rotation_world_flu)


def px4_ned_to_world_state(
    ned_position: np.ndarray,
    ned_velocity: np.ndarray,
    px4_rpy: np.ndarray,
    body_rates_frd: np.ndarray,
    anchor_ned: np.ndarray,
    world_origin: np.ndarray,
    frame_yaw_offset: float,
    t: float,
) -> FlightState:
    """Map PX4 local NED telemetry into the CityNav-style z-up world frame."""

    displacement_enu = _NED_TO_ENU @ (
        np.asarray(ned_position, dtype=np.float64) - np.asarray(anchor_ned, dtype=np.float64)
    )
    velocity_enu = _NED_TO_ENU @ np.asarray(ned_velocity, dtype=np.float64)
    alignment = _rotation_z(frame_yaw_offset)

    position = np.asarray(world_origin, dtype=np.float64) + alignment @ displacement_enu
    velocity = alignment @ velocity_enu
    rpy = px4_attitude_to_world_rpy(px4_rpy, frame_yaw_offset)

    body_rates_flu = _FRD_FROM_FLU @ np.asarray(body_rates_frd, dtype=np.float64)
    return FlightState(
        t=float(t),
        position=position,
        velocity=velocity,
        rpy=rpy,
        angular_velocity=body_rates_flu,
    )


class Px4MavlinkBackend(DynamicsBackend):
    """PX4 SITL/HIL/manual-control backend over pymavlink.

    The backend does not simulate physics itself. PX4 and its connected simulator or
    real vehicle own the dynamics. RealSceneUAV sends MANUAL_CONTROL and maps PX4
    telemetry into the real-scene coordinate frame.
    """

    def __init__(self, config: Px4MavlinkConfig | None = None) -> None:
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise RuntimeError(
                "Install PX4 support with: pip install -e '.[px4]'"
            ) from exc

        self.config = config or Px4MavlinkConfig()
        self._mavutil = mavutil
        self.connection = mavutil.mavlink_connection(
            self.config.connection_url,
            source_system=self.config.source_system,
            source_component=self.config.source_component,
            autoreconnect=True,
        )

        heartbeat = self.connection.wait_heartbeat(
            timeout=self.config.heartbeat_timeout_s
        )
        if heartbeat is None:
            raise TimeoutError(
                f"No PX4 heartbeat on {self.config.connection_url!r} within "
                f"{self.config.heartbeat_timeout_s:.1f} s"
            )

        self.target_system = int(self.connection.target_system)
        self.target_component = int(self.connection.target_component)
        self._heartbeat_info = {
            "type": int(getattr(heartbeat, "type", -1)),
            "autopilot": int(getattr(heartbeat, "autopilot", -1)),
            "system_status": int(getattr(heartbeat, "system_status", -1)),
        }

        self._ned_position: np.ndarray | None = None
        self._ned_velocity: np.ndarray | None = None
        self._px4_rpy: np.ndarray | None = None
        self._body_rates_frd: np.ndarray | None = None
        self._last_position_rx = 0.0
        self._last_attitude_rx = 0.0

        self._anchor_ned = np.zeros(3, dtype=np.float64)
        self._world_origin = np.zeros(3, dtype=np.float64)
        self._frame_yaw_offset = 0.0
        self._time_origin = time.monotonic()
        self._state = FlightState.zero()

        self._request_telemetry()
        self._wait_for_initial_telemetry()

    def supports_state_reset(self) -> bool:
        return False

    def supports_pause_freeze(self) -> bool:
        return False

    def _request_message_interval(self, message_id: int) -> None:
        interval_us = int(round(1_000_000.0 / self.config.telemetry_hz))
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id),
            float(interval_us),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _request_telemetry(self) -> None:
        if self.config.telemetry_hz <= 0:
            raise ValueError("telemetry_hz must be positive")
        self._request_message_interval(
            self._mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
        )
        self._request_message_interval(
            self._mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
        )

    def _update_message(self, message) -> None:
        message_type = message.get_type()
        now = time.monotonic()

        if message_type == "LOCAL_POSITION_NED":
            self._ned_position = np.array(
                [message.x, message.y, message.z],
                dtype=np.float64,
            )
            self._ned_velocity = np.array(
                [message.vx, message.vy, message.vz],
                dtype=np.float64,
            )
            self._last_position_rx = now

        elif message_type == "ATTITUDE":
            self._px4_rpy = np.array(
                [message.roll, message.pitch, message.yaw],
                dtype=np.float64,
            )
            self._body_rates_frd = np.array(
                [message.rollspeed, message.pitchspeed, message.yawspeed],
                dtype=np.float64,
            )
            self._last_attitude_rx = now

    def _drain_telemetry(self) -> None:
        while True:
            message = self.connection.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"],
                blocking=False,
            )
            if message is None:
                return
            self._update_message(message)

    def _wait_for_initial_telemetry(self) -> None:
        deadline = time.monotonic() + self.config.telemetry_timeout_s
        while self._ned_position is None or self._px4_rpy is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    "Timed out waiting for PX4 LOCAL_POSITION_NED and ATTITUDE telemetry"
                )
            message = self.connection.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"],
                blocking=True,
                timeout=remaining,
            )
            if message is not None:
                self._update_message(message)

    def _check_telemetry_freshness(self) -> None:
        now = time.monotonic()
        if now - self._last_position_rx > self.config.telemetry_timeout_s:
            raise RuntimeError("PX4 LOCAL_POSITION_NED telemetry is stale")
        if now - self._last_attitude_rx > self.config.telemetry_timeout_s:
            raise RuntimeError("PX4 ATTITUDE telemetry is stale")

    def _current_world_state(self) -> FlightState:
        if (
            self._ned_position is None
            or self._ned_velocity is None
            or self._px4_rpy is None
            or self._body_rates_frd is None
        ):
            raise RuntimeError("PX4 telemetry is not initialized")

        return px4_ned_to_world_state(
            ned_position=self._ned_position,
            ned_velocity=self._ned_velocity,
            px4_rpy=self._px4_rpy,
            body_rates_frd=self._body_rates_frd,
            anchor_ned=self._anchor_ned,
            world_origin=self._world_origin,
            frame_yaw_offset=self._frame_yaw_offset,
            t=time.monotonic() - self._time_origin,
        )

    def reset(self, state: FlightState | None = None) -> FlightState:
        """Anchor current PX4 pose to the requested world pose without teleporting."""

        self._drain_telemetry()
        self._check_telemetry_freshness()

        desired = state.copy() if state is not None else FlightState.zero()
        self._anchor_ned = self._ned_position.copy()
        self._world_origin = desired.position.copy()

        current_world_rpy = px4_attitude_to_world_rpy(self._px4_rpy, 0.0)
        self._frame_yaw_offset = float(desired.rpy[2] - current_world_rpy[2])
        self._frame_yaw_offset = float(
            (self._frame_yaw_offset + np.pi) % (2.0 * np.pi) - np.pi
        )
        self._time_origin = time.monotonic() - float(desired.t)
        self._state = self._current_world_state()
        return self._state.copy()

    def _send_manual_control(self, command: ControlCommand) -> None:
        x, y, z, r = command_to_px4_manual_control(command)
        self.connection.mav.manual_control_send(
            self.target_system,
            x,
            y,
            z,
            r,
            0,
        )

    def step(self, command: ControlCommand, dt: float) -> FlightState:
        del dt
        self._send_manual_control(command)
        self._drain_telemetry()
        self._check_telemetry_freshness()
        self._state = self._current_world_state()
        return self._state.copy()

    def state(self) -> FlightState:
        self._drain_telemetry()
        self._check_telemetry_freshness()
        self._state = self._current_world_state()
        return self._state.copy()

    def prime_manual_control(
        self,
        duration_s: float = 0.5,
        rate_hz: float = 20.0,
        throttle: float = 0.0,
    ) -> None:
        """Send a safe initial MANUAL_CONTROL stream before mode/arming changes."""

        if duration_s <= 0 or rate_hz <= 0:
            raise ValueError("duration_s and rate_hz must be positive")
        command = ControlCommand(throttle=throttle)
        period = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._send_manual_control(command)
            time.sleep(period)

    def set_mode(self, mode_name: str) -> None:
        mapping = self.connection.mode_mapping()
        if not mapping or mode_name not in mapping:
            available = sorted(mapping or {})
            raise ValueError(
                f"PX4 mode {mode_name!r} is unavailable; available modes: {available}"
            )
        self.connection.set_mode(mapping[mode_name])

    def _wait_armed_state(
        self,
        armed: bool,
        timeout_s: float,
        manual_rate_hz: float = 20.0,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        period = 1.0 / manual_rate_hz
        safe_command = ControlCommand(throttle=0.0)

        while time.monotonic() < deadline:
            self._send_manual_control(safe_command)
            heartbeat = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=False,
            )
            if heartbeat is not None:
                base_mode = int(getattr(heartbeat, "base_mode", 0))
                flag = self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                is_armed = bool(base_mode & flag)
                if is_armed == armed:
                    return
            time.sleep(period)

        state = "armed" if armed else "disarmed"
        raise TimeoutError(f"PX4 did not become {state} within {timeout_s:.1f} s")

    def arm(self, timeout_s: float = 10.0) -> None:
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._wait_armed_state(True, timeout_s)

    def disarm(self, timeout_s: float = 10.0) -> None:
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._wait_armed_state(False, timeout_s)

    def provenance(self) -> dict[str, object]:
        return {
            "backend": type(self).__name__,
            "config": asdict(self.config),
            "target_system": self.target_system,
            "target_component": self.target_component,
            "heartbeat": self._heartbeat_info,
            "coordinate_mapping": {
                "px4_world": "NED",
                "px4_body": "FRD",
                "realsceneuav_world": "ENU-like x=east y=north z=up",
                "realsceneuav_body": "FLU",
                "frame_yaw_offset_rad": self._frame_yaw_offset,
            },
            "state_reset": "re-anchor only; no physical teleport",
        }
