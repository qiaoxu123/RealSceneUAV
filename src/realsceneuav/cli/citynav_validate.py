from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from realsceneuav.scenes.citynav import CityNavTrajectoryAdapter
from realsceneuav.scenes.citynav_raster import CityNavRasterScene, RasterShape


def _pose_to_rpy(pose5d: np.ndarray) -> np.ndarray:
    # CityNav pose ordering is x, y, z, yaw, pitch.
    return np.asarray([0.0, pose5d[4], pose5d[3]], dtype=np.float64)


def _save_topdown(
    scene: CityNavRasterScene,
    trajectory: np.ndarray,
    target: np.ndarray,
    path: Path,
) -> None:
    cv2 = scene._cv2
    image = scene.orthographic_rgb
    height, width = image.shape[:2]
    scale = min(1.0, 1600.0 / max(height, width))
    out_width = max(1, int(round(width * scale)))
    out_height = max(1, int(round(height * scale)))
    canvas = cv2.resize(image, (out_width, out_height), interpolation=cv2.INTER_AREA)

    points = []
    for pose in trajectory:
        col, row = scene.world_to_pixel(float(pose[0]), float(pose[1]))
        points.append([int(round(col * scale)), int(round(row * scale))])
    if len(points) >= 2:
        cv2.polylines(
            canvas,
            [np.asarray(points, dtype=np.int32)],
            isClosed=False,
            color=(255, 255, 255),
            thickness=2,
        )

    start = tuple(points[0]) if points else None
    if start is not None:
        cv2.circle(canvas, start, 7, (0, 255, 0), thickness=-1)

    target_col, target_row = scene.world_to_pixel(float(target[0]), float(target[1]))
    target_px = (int(round(target_col * scale)), int(round(target_row * scale)))
    cv2.circle(canvas, target_px, 8, (255, 0, 0), thickness=-1)

    cv2.imwrite(str(path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate one released CityNav human episode against SensatUrban raster data."
    )
    parser.add_argument("--trajectory-json", required=True)
    parser.add_argument("--episode-index", type=int, required=True)
    parser.add_argument("--rgb-png", required=True)
    parser.add_argument("--height-tif", required=True)
    parser.add_argument("--ground-level", type=float, default=None)
    parser.add_argument("--output", default="outputs/citynav_validate")
    parser.add_argument("--width", type=int, default=256)
    parser.add_argument("--height", type=int, default=256)
    args = parser.parse_args()

    metadata = CityNavTrajectoryAdapter(args.trajectory_json)
    index = args.episode_index
    trajectory = metadata.human_pose_trajectory(index)
    if len(trajectory) == 0:
        raise SystemExit(f"Episode {index} has no human trajectory.")

    task = metadata.navigation_task(index)
    map_name = metadata.map_name(index)
    targets = [
        metadata.target_for_record(i)
        for i in metadata.source_indices_for_map(map_name)
        if metadata.source_record(i).get("target_positions")
    ]

    scene = CityNavRasterScene(
        map_name=map_name,
        rgb_png=args.rgb_png,
        height_tif=args.height_tif,
        targets=targets,
        ground_level_m=args.ground_level,
        output_shape=RasterShape(args.height, args.width),
    )

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    sample_indices = sorted({0, len(trajectory) // 2, len(trajectory) - 1})
    frames = []
    for sample_index in sample_indices:
        pose = trajectory[sample_index]
        observation = scene.observation(pose[:3], _pose_to_rpy(pose))

        rgb_name = f"rgb_{sample_index:06d}.png"
        depth_name = f"depth_{sample_index:06d}.npy"
        scene._cv2.imwrite(
            str(output / rgb_name),
            scene._cv2.cvtColor(observation.rgb, scene._cv2.COLOR_RGB2BGR),
        )
        np.save(output / depth_name, observation.depth)

        frames.append(
            {
                "trajectory_index": sample_index,
                "pose": pose.tolist(),
                "rgb": rgb_name,
                "depth": depth_name,
                "depth_min_m": float(np.nanmin(observation.depth)),
                "depth_max_m": float(np.nanmax(observation.depth)),
            }
        )

    topdown_name = "trajectory_topdown.png"
    _save_topdown(scene, trajectory, task.target.position, output / topdown_name)

    summary = {
        "episode_index": index,
        "map_name": map_name,
        "instruction": task.target.instruction,
        "start_position": task.start_position.tolist(),
        "start_yaw": task.start_yaw,
        "target_position": task.target.position.tolist(),
        "trajectory_length": len(trajectory),
        "ground_level_m": scene.ground_level_m,
        "scene": scene.provenance(),
        "frames": frames,
        "topdown": topdown_name,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2))

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
