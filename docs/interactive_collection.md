# Interactive trajectory collection

This document describes the first human-control workflow.

## 1. Install

```bash
pip install -e ".[controller,dev]"
```

## 2. Inspect the controller

Connect the Switch Pro Controller (or another SDL-compatible gamepad), then run:

```bash
realsceneuav-controller-check --seconds 15
```

Move one stick or press one button at a time. Use the reported axis/button indices to edit
`configs/switch_pro.yaml` if your OS/driver mapping differs.

The default RC-style mapping is:

```text
left stick X   -> yaw
left stick Y   -> throttle
right stick X  -> roll
right stick Y  -> pitch
```

Discrete operator events are kept separate from the continuous sticks:

```text
pause/resume
stop episode
mark target at current UAV pose
reset vehicle to episode start
```

Button indices vary across SDL drivers, so they are configuration values rather than hard-coded
Nintendo labels. The collector reads the YAML mapping at runtime, so the exact axis/button
assignment used for a data-collection session is explicit and reproducible.

## 3. Run a reproducible dry run

No gamepad is required:

```bash
realsceneuav-collect --controller scripted --duration 2 --no-realtime
```

This validates the session, dynamics, scene adapter and recorder.

## 4. Run manual collection

```bash
realsceneuav-collect \
  --controller gamepad \
  --controller-config configs/switch_pro.yaml \
  --duration 300 \
  --control-hz 50 \
  --camera-hz 10
```

The control loop and observation recording rate are intentionally independent.

## 5. Output

Each episode contains:

```text
episode_id/
├── task.json
├── trajectory.csv
├── events.json
├── observations.csv
├── rgb/
│   └── frame_*.npy
└── depth/
    └── frame_*.npy
```

`trajectory.csv` preserves both human command and vehicle response. `observations.csv`
synchronizes visual frames to simulation time.

## 6. Real-scene datasets

The interactive loop does not depend on the mock scene. Replace `MockRealScene` with any
`SceneAdapter` that provides:

- metric coordinate conversion
- target/instruction metadata
- ground-height query
- RGB/depth rendering or retrieval

For CityNav, the current adapter already loads released target and human-trajectory metadata.
The next dataset milestone is to attach SensatUrban/CityNav visual data and validate coordinate
alignment before claiming CityNav-compatible rendering.
