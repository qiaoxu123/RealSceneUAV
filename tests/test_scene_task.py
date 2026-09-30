from realsceneuav.scenes.mock import MockRealScene
from realsceneuav.tasks.sampler import sample_task


def test_task_sampling_is_reproducible():
    scene = MockRealScene()
    a = sample_task(scene, seed=3)
    b = sample_task(scene, seed=3)
    assert a.episode_id == b.episode_id
    assert a.target.target_id == b.target.target_id
