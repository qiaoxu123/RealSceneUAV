from __future__ import annotations

import argparse
import time


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect SDL joystick axes/buttons before editing configs/switch_pro.yaml."
    )
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    try:
        import pygame
    except ImportError as exc:
        raise SystemExit("Install controller support with: pip install -e '.[controller]'") from exc

    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        raise SystemExit("No SDL joystick/gamepad detected.")
    if args.index >= count:
        raise SystemExit(f"Requested joystick index {args.index}, but only {count} detected.")

    joystick = pygame.joystick.Joystick(args.index)
    joystick.init()
    print(f"name={joystick.get_name()!r}")
    print(f"axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()} hats={joystick.get_numhats()}")
    print("Move one stick / press one button at a time. Ctrl-C to stop.")

    end = time.monotonic() + args.seconds
    previous_axes = [0.0] * joystick.get_numaxes()
    previous_buttons = [0] * joystick.get_numbuttons()
    while time.monotonic() < end:
        pygame.event.pump()
        axes = [round(float(joystick.get_axis(i)), 3) for i in range(joystick.get_numaxes())]
        buttons = [int(joystick.get_button(i)) for i in range(joystick.get_numbuttons())]

        axis_changes = [
            (i, value)
            for i, value in enumerate(axes)
            if abs(value - previous_axes[i]) >= 0.15
        ]
        button_changes = [
            (i, value)
            for i, value in enumerate(buttons)
            if value != previous_buttons[i]
        ]
        if axis_changes or button_changes:
            print(f"axes={axis_changes} buttons={button_changes}")

        previous_axes = axes
        previous_buttons = buttons
        time.sleep(0.05)


if __name__ == "__main__":
    main()
