# RealSceneUAV

**Real-world scene data + realistic UAV dynamics + human/gamepad control + reproducible trajectory collection.**

RealSceneUAV is designed for aerial navigation research where the **environment comes from real captured scenes** (point clouds, RGB-D, meshes, 3D Gaussian Splatting, maps and target annotations), while the vehicle state evolves through a replaceable UAV dynamics backend.

It is not intended to be another synthetic city simulator. The project separates **scene data**, **vehicle dynamics**, **human control**, **task generation**, and **episode recording** so that the same flight stack can be used with CityNav/SensatUrban-like datasets or new real-world captures.

## Why this project?

CityNav's CityFlight showed that real reconstructed city data can support human aerial navigation collection, but the public CityNav repository does not provide a reusable CityFlight implementation. Existing open-source drone simulators provide excellent physics, rendering, or PX4 integration, but they are generally not organized around **dataset adapters + language targets + human demonstration collection over real captured scenes**.

RealSceneUAV focuses on that missing layer.

## Design goals

- **Real-scene first**: point cloud / RGB-D / mesh / 3DGS data should be loaded through adapters rather than rebuilt as synthetic worlds.
- **Gamepad-first human control**: support Switch Pro Controller/Joy-Con and other SDL gamepads with continuous stick input.
- **Replaceable dynamics**: a small deterministic reference model for CI, then PX4 SITL/HIL, RotorPy, Project AirSim, or other validated backends for experiments.
- **Dataset-neutral**: CityNav is one adapter, not the architecture.
- **Full demonstrations**: save raw stick input, state, target, task metadata, events, and later RGB/depth frames.
- **Reproducible episodes**: seeded task sampling, explicit configuration, versioned schemas, and deterministic CI path.

## Architecture

```text
Real captured data
(point cloud / RGB-D / mesh / 3DGS / OSM / annotations)
                         |
                         v
                  +---------------+
                  | SceneAdapter  |
                  +---------------+
                         |
         +---------------+---------------+
         |                               |
         v                               v
 +---------------+                +---------------+
 | Task / Target |                | Observation   |
 |   Manager     |                | RGB / Depth   |
 +---------------+                +---------------+
         |                               ^
         v                               |
 +---------------+     commands    +---------------+
 | Switch / SDL  | --------------> | Dynamics      |
 | Controller    |                 | Backend       |
 +---------------+                 +---------------+
                                          |
                                          v
                                  +---------------+
                                  | Flight State  |
                                  +---------------+
                                          |
                                          v
                                  +---------------+
                                  | Recorder      |
                                  +---------------+
```

### Core interfaces

**`SceneAdapter`**
- converts dataset coordinates into a common world frame
- exposes target objects and instructions
- returns RGB/depth observations from the real-scene representation
- provides ground height / geometry queries

**`DynamicsBackend`**
- consumes normalized human/autonomous control commands
- advances the UAV state
- can be replaced without changing task or dataset code

**`Controller`**
- Switch Pro / Joy-Con / Xbox / generic SDL device
- scripted expert
- learned policy
- future PX4/MAVLink command source

**`EpisodeRecorder`**
- task metadata
- synchronized flight state
- raw human control input
- distance-to-target and events
- later: RGB/depth, target marker, collision and human annotations

## Current v0.1 scope

The first commit intentionally keeps the dependency surface small and provides:

- deterministic 6-DoF reference quadrotor backend
- generic SDL/pygame gamepad input
- Switch Pro Controller mapping template
- dataset-neutral `SceneAdapter`
- target/task abstractions
- synchronized trajectory recorder
- mock real-scene adapter for CI
- CityNav MTurk trajectory JSON metadata adapter
- unit tests and GitHub Actions CI

> The reference dynamics are **not claimed to be a validated real UAV model**. Their purpose is to make the full pipeline runnable everywhere. Research runs should use a validated backend such as PX4 SITL/HIL or RotorPy.

## Installation

Python 3.10+ is recommended.

```bash
git clone https://github.com/qiaoxu123/RealSceneUAV.git
cd RealSceneUAV
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest -q
```

For Switch/gamepad support:

```bash
pip install -e ".[controller]"
realsceneuav-controller-check --seconds 15
```

Run a headless reproducible collection smoke test:

```bash
realsceneuav-collect --controller scripted --duration 2 --no-realtime
```

Run an interactive gamepad collection:

```bash
realsceneuav-collect --controller gamepad --duration 300 --control-hz 50 --camera-hz 10
```

For future MAVLink/PX4 integration:

```bash
pip install -e ".[px4]"
```

## Reproducible smoke test

```bash
realsceneuav-demo --output outputs --seconds 2
```

This creates:

```text
outputs/mock-real-scene-000000/
├── task.json
├── trajectory.csv
└── events.json
```

`trajectory.csv` records synchronized:

```text
t, x, y, z,
vx, vy, vz,
roll, pitch, yaw,
roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd,
distance_to_target_m
```

## Human control convention

The default Switch-style mapping follows common RC semantics:

```text
Left stick X   -> yaw
Left stick Y   -> throttle
Right stick X  -> roll
Right stick Y  -> pitch
```

Raw stick commands are normalized and recorded. This is deliberately different from CityFlight-style discrete `forward / left / right / up / down` actions because human demonstrations should retain continuous control intent.

The mapping is configurable in:

```text
configs/switch_pro.yaml
```

## Dataset integration

A new dataset implements `SceneAdapter` rather than modifying the flight loop:

```python
class MyDatasetAdapter(SceneAdapter):
    def scene_id(self) -> str: ...
    def sample_targets(self) -> list[Target]: ...
    def observation(self, position, rpy) -> Observation: ...
    def ground_height(self, x, y) -> float: ...
```

### CityNav

The repository already includes a metadata adapter for released CityNav MTurk trajectory JSON files:

```python
from realsceneuav.scenes.citynav import CityNavTrajectoryAdapter

scene = CityNavTrajectoryAdapter(
    "data/citynav/citynav_train_seen.json",
    ground_height_m=0.0,
)
targets = scene.sample_targets()
human_traj = scene.human_trajectory(0)
```

This exposes target positions, descriptions, scene identifiers, annotation IDs and human pose trajectories. Visual rendering is intentionally kept separate because it should come from the corresponding real SensatUrban RGB-D / point-cloud / 3DGS representation.

Planned adapters:

1. **CityNav / SensatUrban**
   - CityNav instructions, target objects and human trajectories
   - SensatUrban point cloud / rasterized RGB-D
   - OSM landmarks when available
2. **3D Gaussian Splatting scenes**
   - real captured scene rendering via `gsplat`
3. **Generic RGB-D / point-cloud datasets**
   - local frame registration and target manifests

## Planned data schema

Each recorded episode will converge on:

```text
episode_xxxxxx/
├── task.json
├── trajectory.csv
├── events.json
├── rgb/
├── depth/
└── metadata.json
```

The long-term schema preserves three different signals instead of collapsing them:

1. **human input**: sticks/buttons
2. **vehicle response**: pose, velocity, attitude, acceleration
3. **visual state**: RGB, depth, visible targets/landmarks

This makes the data useful for behavior cloning, VLA training, landmark-arrival detection, active perception, and human flight-strategy analysis.

## Comparison with existing open-source projects

RealSceneUAV should reuse strong components rather than replace them.

| Project | Strongest capability | Gap relative to our target |
|---|---|---|
| PX4 SITL/HIL | production flight stack and realistic controller integration | not a real-scene dataset / language-trajectory platform |
| Project AirSim / AirSim | mature vehicle simulation and sensors | not organized around real dataset adapters and human collection |
| Pegasus Simulator | PX4 + Isaac Sim robotics integration | primarily simulator/Isaac workflow |
| RotorPy | lightweight multirotor dynamics, controllers and wind | no real-scene language-navigation data layer |
| Flightmare | fast quadrotor RL simulation | synthetic Unity-centered rendering workflow |
| GS-DroneGym | closest project: 3DGS rendering, drone dynamics, RGB/depth, viewer and dataset tooling | no PX4/gamepad-first CityNav-style human collection layer |
| **RealSceneUAV** | real-data adapters + continuous human control + replaceable dynamics + demonstration collection | this repository |

### Closest existing project: GS-DroneGym

GS-DroneGym is close enough that we should learn from it rather than duplicate it. Its public implementation already provides:

- Gaussian-splat rendering
- RGB/depth observations
- quadrotor dynamics
- collision geometry derived from Gaussians
- manual viewer
- trajectory/dataset tooling

RealSceneUAV therefore should **not** spend effort reproducing all of that. Its intended differentiation is:

- explicit support for external real-world navigation datasets such as CityNav/SensatUrban
- Switch/gamepad continuous human control as a first-class input
- PX4-compatible dynamics/control path
- task/target selection and human demonstration collection workflow
- preservation of raw human controls and vehicle response
- explicit scene provenance, coordinate frame, target origin and collection provenance

## Roadmap

### M0 - repository foundation (current)

- [x] common state/control schema
- [x] reference dynamics backend
- [x] gamepad abstraction
- [x] scene adapter abstraction
- [x] target/task abstraction
- [x] episode recorder
- [x] CityNav trajectory metadata adapter
- [x] deterministic tests and CI

### M1 - interactive collection

- [ ] live 3D viewer
- [x] Switch Pro Controller inspection/calibration CLI
- [x] pause/reset/mark-target buttons
- [x] RGB/depth recording at an independent camera rate
- [x] explicit `STOP` and target-marker events
- [x] real-time manual collection loop

See [Interactive trajectory collection](docs/interactive_collection.md).

### M2 - CityNav/SensatUrban adapter

- [ ] load CityRefer objects/descriptions
- [ ] coordinate alignment and ground-height query
- [ ] reproduce CityNav start/target tasks
- [ ] OSM landmark layer
- [ ] import/replay existing human demonstrations

### M3 - real-scene renderer

- [ ] 3DGS renderer backend
- [ ] point-cloud viewer / rasterized RGB-D backend
- [ ] scene-scale and coordinate validation
- [ ] collision/visibility queries from real geometry

### M4 - validated flight dynamics

- [ ] PX4 SITL bridge
- [ ] MAVLink manual-control bridge
- [ ] RotorPy backend
- [ ] vehicle profile files (mass, inertia, thrust limits)
- [ ] separate simulation/control/camera clocks

### M5 - benchmark and dataset release

- [ ] seen/unseen split manifests
- [ ] deterministic target/start sampling
- [ ] human subject/session metadata without personal identifiers
- [ ] dataset validator and replay tool
- [ ] export to Parquet / LeRobot-style format

## Reproducibility principles

Every benchmark release should record:

- repository commit
- adapter version
- scene/data source and checksum
- coordinate frame and units
- target manifest version
- vehicle backend and parameters
- controller mapping
- random seed
- control/physics/camera rates

Do not treat a rendered reconstruction as ground-truth physics. Scene geometry provenance and vehicle-dynamics provenance must remain separate.

## License

Code is released under the MIT License. External datasets, PX4, CityNav/SensatUrban, and third-party renderers retain their own licenses and must be downloaded/used according to their respective terms.
