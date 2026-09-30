from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from realsceneuav.scenes.base import Observation, SceneAdapter, Target
from realsceneuav.scenes.citynav_mapdata import citynav_ground_level


@dataclass(frozen=True, slots=True)
class RasterShape:
    height: int = 256
    width: int = 256


def citynav_view_area_world(
    position: np.ndarray,
    yaw: float,
    ground_level_m: float,
) -> np.ndarray:
    """Match CityNav's square aerial view footprint in world XY coordinates."""

    center = np.asarray(position[:2], dtype=np.float64)
    altitude = float(position[2]) - float(ground_level_m)
    if altitude <= 0:
        raise ValueError(
            f"UAV altitude must be above ground level ({ground_level_m:.3f} m)"
        )

    front = np.array([np.cos(yaw), np.sin(yaw)], dtype=np.float64)
    left = np.array([-np.sin(yaw), np.cos(yaw)], dtype=np.float64)
    return np.stack(
        [
            center + altitude * (front + left),
            center + altitude * (front - left),
            center + altitude * (-front - left),
            center + altitude * (-front + left),
        ]
    )


class CityNavRasterScene(SceneAdapter):
    """CityNav-compatible observation adapter over real SensatUrban raster data.

    The implementation follows CityNav's released cropclient semantics:
    a pose-dependent square footprint is projected into the orthographic RGB and
    height rasters, then depth is computed as UAV z minus raster surface height.
    """

    def __init__(
        self,
        map_name: str,
        rgb_png: str | Path,
        height_tif: str | Path,
        targets: list[Target] | None = None,
        ground_level_m: float | None = None,
        output_shape: RasterShape | tuple[int, int] = RasterShape(),
    ) -> None:
        try:
            import cv2
            import rasterio
        except ImportError as exc:
            raise RuntimeError(
                "Install CityNav raster support with: pip install -e '.[citynav]'"
            ) from exc

        self._cv2 = cv2
        self._rasterio = rasterio
        self.map_name = str(map_name)
        self.rgb_path = Path(rgb_png)
        self.height_path = Path(height_tif)
        self._targets = list(targets or [])
        self.ground_level_m = (
            citynav_ground_level(self.map_name)
            if ground_level_m is None
            else float(ground_level_m)
        )

        if isinstance(output_shape, RasterShape):
            self.output_shape = output_shape
        else:
            self.output_shape = RasterShape(height=int(output_shape[0]), width=int(output_shape[1]))
        if self.output_shape.height <= 0 or self.output_shape.width <= 0:
            raise ValueError("output_shape must be positive")

        bgr = cv2.imread(str(self.rgb_path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise FileNotFoundError(f"Could not read RGB raster: {self.rgb_path}")
        self._rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        with rasterio.open(self.height_path) as src:
            self._height = src.read(1).astype(np.float32)
            self._transform = src.transform
            self._crs = None if src.crs is None else src.crs.to_string()
            self._bounds = tuple(float(value) for value in src.bounds)

        if self._rgb.shape[:2] != self._height.shape:
            raise ValueError(
                "RGB and height raster dimensions differ: "
                f"{self._rgb.shape[:2]} vs {self._height.shape}"
            )

    def scene_id(self) -> str:
        return self.map_name

    def sample_targets(self) -> list[Target]:
        return list(self._targets)

    def provenance(self) -> dict[str, Any]:
        return {
            "scene_id": self.map_name,
            "adapter": type(self).__name__,
            "rgb_png": str(self.rgb_path.resolve()),
            "height_tif": str(self.height_path.resolve()),
            "ground_level_m": self.ground_level_m,
            "raster_shape": list(self._height.shape),
            "observation_shape": [self.output_shape.height, self.output_shape.width],
            "crs": self._crs,
            "bounds": list(self._bounds),
            "source": "SensatUrban/CityNav raster workflow",
        }

    def ground_height(self, x: float, y: float) -> float:
        # CityNav uses one reference ground level per block for view geometry.
        return self.ground_level_m

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        col, row = (~self._transform) * (float(x), float(y))
        return float(col), float(row)

    def _view_area_world(self, position: np.ndarray, yaw: float) -> np.ndarray:
        return citynav_view_area_world(position, yaw, self.ground_level_m)

    def _view_area_pixels(self, position: np.ndarray, yaw: float) -> np.ndarray:
        corners = self._view_area_world(position, yaw)
        return np.asarray(
            [self.world_to_pixel(float(x), float(y)) for x, y in corners],
            dtype=np.float32,
        )

    def observation(self, position: np.ndarray, rpy: np.ndarray) -> Observation:
        position = np.asarray(position, dtype=np.float64)
        rpy = np.asarray(rpy, dtype=np.float64)
        if position.shape != (3,) or rpy.shape != (3,):
            raise ValueError("position and rpy must both have shape (3,)")

        src = self._view_area_pixels(position, float(rpy[2]))
        height, width = self.output_shape.height, self.output_shape.width
        dst = np.asarray(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
            dtype=np.float32,
        )
        transform = self._cv2.getPerspectiveTransform(src, dst)

        rgb = self._cv2.warpPerspective(
            self._rgb,
            transform,
            (width, height),
            flags=self._cv2.INTER_LINEAR,
        )
        surface_height = self._cv2.warpPerspective(
            self._height,
            transform,
            (width, height),
            flags=self._cv2.INTER_LINEAR,
        )
        depth = (float(position[2]) - surface_height).astype(np.float32)

        return Observation(
            rgb=rgb,
            depth=depth,
            metadata={
                "map_name": self.map_name,
                "position": position.tolist(),
                "rpy": rpy.tolist(),
                "view_area_world": self._view_area_world(position, float(rpy[2])).tolist(),
            },
        )

    @property
    def orthographic_rgb(self) -> np.ndarray:
        """Return a copy of the full real-scene orthographic RGB raster."""

        return self._rgb.copy()
