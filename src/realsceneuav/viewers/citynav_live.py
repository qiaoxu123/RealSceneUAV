from __future__ import annotations

import textwrap
from typing import Any

import numpy as np

from realsceneuav.core.observer import SessionObserver
from realsceneuav.core.types import ControlCommand, FlightState
from realsceneuav.scenes.base import Observation, SceneAdapter
from realsceneuav.scenes.citynav_raster import CityNavRasterScene
from realsceneuav.tasks.navigation import NavigationTask


class CityNavLiveViewer(SessionObserver):
    """Pygame live viewer for real CityNav/SensatUrban raster navigation."""

    def __init__(
        self,
        scene: CityNavRasterScene,
        task: NavigationTask,
        panel_size: int = 420,
    ) -> None:
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError(
                "Install viewer/controller support with: pip install -e '.[controller,citynav]'"
            ) from exc

        if panel_size < 200:
            raise ValueError("panel_size must be at least 200 pixels")

        self.pygame = pygame
        self.scene = scene
        self.task = task
        self.panel_size = int(panel_size)
        self.status = "ready"
        self.paused = False
        self.active = True
        self.latest_state: FlightState | None = None
        self.trajectory_xy: list[np.ndarray] = []

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode(
            (self.panel_size * 3, self.panel_size + 120)
        )
        pygame.display.set_caption("RealSceneUAV - CityNav Live")
        self.font = pygame.font.SysFont(None, 22)
        self.small_font = pygame.font.SysFont(None, 18)

        ortho = scene.orthographic_rgb
        self.map_height, self.map_width = ortho.shape[:2]
        self.map_surface = pygame.transform.smoothscale(
            self._rgb_surface(ortho),
            (self.panel_size, self.panel_size),
        )
        self.map_scale_x = self.panel_size / self.map_width
        self.map_scale_y = self.panel_size / self.map_height

    def _pump_window_events(self) -> None:
        if not self.active:
            return
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                self.active = False
                self.pygame.display.quit()
                break

    def _rgb_surface(self, rgb: np.ndarray):
        array = np.asarray(rgb)
        if array.ndim != 3 or array.shape[2] != 3:
            raise ValueError("RGB observation must have shape H x W x 3")
        array = np.ascontiguousarray(np.clip(array, 0, 255).astype(np.uint8))
        return self.pygame.surfarray.make_surface(np.swapaxes(array, 0, 1))

    def _depth_surface(self, depth: np.ndarray):
        values = np.asarray(depth, dtype=np.float32).squeeze()
        if values.ndim != 2:
            raise ValueError("Depth observation must be 2D or H x W x 1")

        finite = np.isfinite(values)
        if finite.any():
            valid = values[finite]
            lo = float(np.percentile(valid, 5.0))
            hi = float(np.percentile(valid, 95.0))
            if hi <= lo:
                hi = lo + 1.0
            normalized = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
            gray = (255.0 * (1.0 - normalized)).astype(np.uint8)
            gray[~finite] = 0
        else:
            gray = np.zeros_like(values, dtype=np.uint8)

        rgb = np.repeat(gray[..., None], 3, axis=-1)
        return self._rgb_surface(rgb)

    def _map_point(self, x: float, y: float) -> tuple[int, int]:
        col, row = self.scene.world_to_pixel(x, y)
        px = int(round(col * self.map_scale_x)) + 2 * self.panel_size
        py = int(round(row * self.map_scale_y))
        return px, py

    def _draw_topdown(self, state: FlightState) -> None:
        pygame = self.pygame
        self.screen.blit(self.map_surface, (2 * self.panel_size, 0))

        if len(self.trajectory_xy) >= 2:
            points = [
                self._map_point(float(point[0]), float(point[1]))
                for point in self.trajectory_xy
            ]
            pygame.draw.lines(self.screen, (255, 255, 255), False, points, 2)

        target = self.task.target.position
        pygame.draw.circle(
            self.screen,
            (255, 60, 60),
            self._map_point(float(target[0]), float(target[1])),
            7,
        )

        current = self._map_point(float(state.position[0]), float(state.position[1]))
        pygame.draw.circle(self.screen, (50, 255, 80), current, 7)

        yaw = float(state.rpy[2])
        heading_length = 24
        endpoint = (
            int(round(current[0] + heading_length * np.cos(yaw))),
            int(round(current[1] + heading_length * np.sin(yaw))),
        )
        pygame.draw.line(self.screen, (50, 255, 80), current, endpoint, 3)

    def _draw_label(self, text: str, x: int, y: int) -> None:
        surface = self.small_font.render(text, True, (255, 255, 255))
        self.screen.blit(surface, (x, y))

    def _draw_footer(self, state: FlightState) -> None:
        pygame = self.pygame
        footer_y = self.panel_size
        pygame.draw.rect(
            self.screen,
            (20, 20, 20),
            (0, footer_y, self.panel_size * 3, 120),
        )

        distance = self.task.distance_to_target(state.position)
        telemetry = (
            f"t={state.t:.1f}s  "
            f"xyz=({state.position[0]:.1f}, {state.position[1]:.1f}, "
            f"{state.position[2]:.1f})  "
            f"yaw={np.degrees(state.rpy[2]):.1f} deg  "
            f"target={distance:.1f} m  "
            f"status={self.status}"
        )
        self.screen.blit(
            self.font.render(telemetry, True, (235, 235, 235)),
            (10, footer_y + 8),
        )

        instruction = f"Instruction: {self.task.target.instruction}"
        lines = textwrap.wrap(instruction, width=130)[:3]
        for index, line in enumerate(lines):
            self.screen.blit(
                self.small_font.render(line, True, (230, 230, 230)),
                (10, footer_y + 38 + 22 * index),
            )

    def _draw(self, state: FlightState, observation: Observation) -> None:
        if not self.active or observation.rgb is None or observation.depth is None:
            return

        self._pump_window_events()
        if not self.active:
            return

        rgb_surface = self.pygame.transform.smoothscale(
            self._rgb_surface(observation.rgb),
            (self.panel_size, self.panel_size),
        )
        depth_surface = self.pygame.transform.smoothscale(
            self._depth_surface(observation.depth),
            (self.panel_size, self.panel_size),
        )

        self.screen.fill((0, 0, 0))
        self.screen.blit(rgb_surface, (0, 0))
        self.screen.blit(depth_surface, (self.panel_size, 0))
        self._draw_topdown(state)

        self._draw_label("RGB", 8, 8)
        self._draw_label("Depth", self.panel_size + 8, 8)
        self._draw_label("Top-down", 2 * self.panel_size + 8, 8)
        self._draw_footer(state)

        self.pygame.display.flip()

    def on_start(
        self,
        scene: SceneAdapter,
        task: NavigationTask,
        state: FlightState,
    ) -> None:
        self.latest_state = state
        self.trajectory_xy = [state.position[:2].copy()]
        self.status = "running"

    def on_state(
        self,
        state: FlightState,
        command: ControlCommand,
        paused: bool,
    ) -> None:
        del command
        self.latest_state = state
        self.paused = paused
        self._pump_window_events()

        point = state.position[:2].copy()
        if not self.trajectory_xy or np.linalg.norm(point - self.trajectory_xy[-1]) >= 0.25:
            self.trajectory_xy.append(point)

    def on_observation(
        self,
        state: FlightState,
        observation: Observation,
    ) -> None:
        self.latest_state = state
        self._draw(state, observation)

    def on_event(
        self,
        event_type: str,
        state: FlightState,
        payload: dict[str, Any],
    ) -> None:
        del state, payload
        self.status = event_type

    def close(self) -> None:
        if self.active:
            self.pygame.display.quit()
        self.active = False
