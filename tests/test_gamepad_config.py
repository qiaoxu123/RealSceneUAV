from realsceneuav.controllers.gamepad import load_gamepad_config


def test_load_gamepad_config(tmp_path):
    path = tmp_path / "switch.yaml"
    path.write_text(
        """
controller:
  type: pygame
  joystick_index: 2
  mapping:
    yaw_axis: 4
    throttle_axis: 5
    roll_axis: 0
    pitch_axis: 1
    invert_yaw: true
    deadzone: 0.1
    mark_target_button: 7
    stop_button: 8
    reset_button: 9
    pause_button: 10
"""
    )

    mapping, joystick_index = load_gamepad_config(path)

    assert joystick_index == 2
    assert mapping.yaw_axis == 4
    assert mapping.throttle_axis == 5
    assert mapping.invert_yaw is True
    assert mapping.deadzone == 0.1
    assert mapping.mark_target_button == 7
    assert mapping.pause_button == 10
