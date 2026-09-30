import json

import numpy as np

from realsceneuav.scenes.citynav import CityNavTrajectoryAdapter


def test_citynav_trajectory_adapter(tmp_path):
    path = tmp_path / "citynav.json"
    path.write_text(
        json.dumps(
            [
                {
                    "area": "cambridge",
                    "block": "0",
                    "object_ids": [7],
                    "ann_ids": [11],
                    "descriptions": ["the target building"],
                    "trajectory": [[0, 0, 10, 0, 0], [1, 0, 10, 0, 0]],
                    "target_positions": [[20, 30, 5]],
                    "split": "train_seen",
                }
            ]
        )
    )

    adapter = CityNavTrajectoryAdapter(path)
    targets = adapter.sample_targets()
    assert len(targets) == 1
    assert targets[0].instruction == "the target building"
    assert np.allclose(targets[0].position, [20, 30, 5])
    assert adapter.human_trajectory(0).shape == (2, 5)
