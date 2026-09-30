import json

import numpy as np

from realsceneuav.scenes.citynav import CityNavTrajectoryAdapter
from realsceneuav.scenes.citynav_mapdata import citynav_ground_level
from realsceneuav.scenes.citynav_raster import citynav_view_area_world


def test_citynav_trajectory_adapter_reconstructs_task(tmp_path):
    path = tmp_path / "citynav.json"
    path.write_text(
        json.dumps(
            [
                {
                    "area": "cambridge",
                    "block": "2",
                    "object_ids": [7],
                    "ann_ids": [11],
                    "descriptions": ["the target building"],
                    "trajectory": [[0, 0, 50, 0.25, -0.1], [1, 0, 50, 0.25, -0.1]],
                    "target_positions": [[20, 30, 45]],
                    "marker_positions": [[19, 29, 45]],
                    "split": "train_seen",
                }
            ]
        )
    )

    adapter = CityNavTrajectoryAdapter(path)
    targets = adapter.sample_targets()
    task = adapter.navigation_task(0)

    assert len(targets) == 1
    assert targets[0].instruction == "the target building"
    assert targets[0].metadata["map_name"] == "cambridge_block_2"
    assert np.allclose(targets[0].position, [20, 30, 45])

    assert adapter.map_name(0) == "cambridge_block_2"
    assert adapter.human_trajectory(0).shape == (2, 5)
    assert adapter.human_pose_trajectory(0).shape == (2, 5)
    assert np.allclose(task.start_position, [0, 0, 50])
    assert task.start_yaw == 0.25
    assert np.allclose(task.target.position, [20, 30, 45])


def test_citynav_direction_vector_pose_is_normalized(tmp_path):
    path = tmp_path / "citynav.json"
    path.write_text(
        json.dumps(
            [
                {
                    "area": "birmingham",
                    "block": "0",
                    "object_ids": [1],
                    "ann_ids": [2],
                    "descriptions": ["target"],
                    "trajectory": [[1, 2, 30, 0, 1, 0]],
                    "target_positions": [[4, 5, 6]],
                }
            ]
        )
    )

    adapter = CityNavTrajectoryAdapter(path)
    pose = adapter.human_pose_trajectory(0)[0]

    assert np.allclose(pose[:3], [1, 2, 30])
    assert np.isclose(pose[3], np.pi / 2)
    assert np.isclose(pose[4], 0.0)


def test_citynav_ground_level_matches_official_metadata():
    assert np.isclose(citynav_ground_level("cambridge_block_2"), 37.619963888870764)
    assert np.isclose(citynav_ground_level("birmingham_block_0"), 16.048856444156698)


def test_citynav_view_area_matches_official_square_geometry():
    position = np.array([100.0, 200.0, 60.0])
    ground = 40.0

    corners = citynav_view_area_world(position, yaw=0.0, ground_level_m=ground)

    assert np.allclose(
        corners,
        [
            [120.0, 220.0],
            [120.0, 180.0],
            [80.0, 180.0],
            [80.0, 220.0],
        ],
    )
