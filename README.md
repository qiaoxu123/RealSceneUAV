# RealSceneUAV

[English](README_EN.md)

**真实场景数据 + UAV 飞行动力学 + Switch/手柄遥控 + 可复现 Human Trajectory 采集。**

RealSceneUAV 面向语言引导无人机导航与 Human Demonstration 数据采集。与传统合成仿真器不同，本项目强调：

> **场景来自真实采集/重建数据，只有无人机运动过程由可替换的飞行动力学后端计算。**

因此项目的基本形式是：

```text
真实场景数据
Point Cloud / RGB-D / Mesh / 3DGS / OSM
                    |
                    v
              SceneAdapter
                    |
         +----------+----------+
         |                     |
         v                     v
   Task / Target          RGB / Depth
         |                     ^
         v                     |
 Switch / Gamepad ---> UAV Dynamics
                             |
                             v
                        Flight State
                             |
                             v
                         Recorder
```

## 1. 为什么做这个项目？

CityNav 的 CityFlight 证明了可以在真实城市重建数据上采集人类无人机导航轨迹，但目前公开的 CityNav 仓库并没有提供可直接复用的 CityFlight 完整实现。

另一方面，PX4、AirSim、Pegasus、RotorPy、Flightmare、GS-DroneGym 等项目已经提供了较成熟的动力学、飞控或渲染能力。

RealSceneUAV 不重复造一个新的“AirSim”，而是重点补齐：

- **真实数据集场景接口**；
- **语言目标与目标选择**；
- **Switch/手柄连续遥控**；
- **真实/可替换 UAV 动力学**；
- **Human Trajectory 完整采集**；
- **不同数据集统一接入与复现**。

## 2. 当前已经实现

### M0：基础框架

- [x] 统一 `FlightState` / `ControlCommand`
- [x] 可替换 `DynamicsBackend`
- [x] 可替换 `SceneAdapter`
- [x] Target / Task 抽象
- [x] CityNav MTurk trajectory JSON 读取
- [x] 可复现测试与 GitHub Actions CI

### M1：交互式 Human Trajectory 采集

- [x] Switch Pro Controller / SDL 通用手柄
- [x] 连续 Roll / Pitch / Yaw / Throttle 输入
- [x] Pause / Resume
- [x] Stop
- [x] Reset
- [x] Mark Target
- [x] 控制频率与相机采样频率解耦
- [x] RGB / Depth 同步记录
- [x] trajectory / event / metadata 完整记录
- [x] 手柄映射检查工具

详细说明见：

[docs/interactive_collection.md](docs/interactive_collection.md)

## 3. 安装

推荐 Python 3.10+。

```bash
git clone https://github.com/qiaoxu123/RealSceneUAV.git
cd RealSceneUAV

python -m venv .venv
source .venv/bin/activate

pip install -e ".[dev]"
pytest -q
```

安装手柄支持：

```bash
pip install -e ".[controller]"
```

## 4. Switch / 手柄控制

默认采用常见 RC 控制语义：

```text
Left Stick X   -> Yaw
Left Stick Y   -> Throttle

Right Stick X  -> Roll
Right Stick Y  -> Pitch
```

先检测当前系统中的 SDL 映射：

```bash
realsceneuav-controller-check --seconds 15
```

然后修改：

```text
configs/switch_pro.yaml
```

再运行：

```bash
realsceneuav-collect \
  --controller gamepad \
  --controller-config configs/switch_pro.yaml \
  --duration 300 \
  --control-hz 50 \
  --camera-hz 10
```

## 5. 无手柄可复现测试

```bash
realsceneuav-collect \
  --controller scripted \
  --duration 2 \
  --no-realtime
```

会生成：

```text
outputs/<episode_id>/
├── task.json
├── trajectory.csv
├── observations.csv
├── events.json
├── metadata.json
├── rgb/
└── depth/
```

其中：

- `task.json`：起点、目标、语言指令；
- `trajectory.csv`：位置、速度、姿态、原始控制量；
- `observations.csv`：视觉帧与时间戳；
- `events.json`：Stop / Reset / Marker 等事件；
- `metadata.json`：场景、采样频率、手柄映射、动力学后端及参数。

## 6. CityNav / SensatUrban

RealSceneUAV 已经能够读取 CityNav 发布的 Human Trajectory JSON：

```python
from realsceneuav.scenes.citynav import CityNavTrajectoryAdapter

scene = CityNavTrajectoryAdapter(
    "data/citynav/citynav_train_seen.json"
)

targets = scene.sample_targets()
human_traj = scene.human_trajectory(0)
```

当前 M2 正在进一步接入 CityNav 官方使用的真实视觉链：

```text
SensatUrban UAV 航拍
        |
        v
真实城市 3D 重建
        |
        +--> 正射 RGB
        |
        +--> Height GeoTIFF
                  |
                  v
             UAV Pose
                  |
                  v
         Perspective Crop
                  |
          +-------+-------+
          |               |
         RGB          Depth = z - height
```

这里的 RGB / Depth 来自真实采集城市数据，而不是合成游戏场景。

安装 CityNav raster 支持：

```bash
pip install -e ".[citynav]"
```

验证一条真实 Human Trajectory 与 raster 坐标：

```bash
realsceneuav-citynav-validate \
  --trajectory-json data/citynav_train_seen.json \
  --episode-index 0 \
  --rgb-png data/rgbd/cambridge_block_2.png \
  --height-tif data/rgbd/cambridge_block_2.tif \
  --output outputs/citynav_episode_0
```

详细说明：

[docs/citynav_raster.md](docs/citynav_raster.md)

## 7. 数据集扩展方式

新的真实场景数据集只需要实现 `SceneAdapter`：

```python
class MyDatasetAdapter(SceneAdapter):
    def scene_id(self) -> str: ...
    def sample_targets(self) -> list[Target]: ...
    def observation(self, position, rpy) -> Observation: ...
    def ground_height(self, x, y) -> float: ...
```

未来计划支持：

- CityNav / SensatUrban
- 自采 UAV 点云
- RGB-D 地图
- Mesh
- 3D Gaussian Splatting
- 其他真实城市场景导航数据集

## 8. 与已有开源方案的关系

| 项目 | 优势 | RealSceneUAV 的区别 |
|---|---|---|
| PX4 SITL/HIL | 成熟飞控与飞行动力学 | 缺少真实导航数据集/Human Trajectory 数据层 |
| Project AirSim / AirSim | 完整 UAV 仿真和传感器 | 主要面向构建仿真世界 |
| Pegasus | Isaac Sim + PX4 | 重点是机器人仿真 |
| RotorPy | 轻量多旋翼动力学 | 没有真实场景导航数据接口 |
| Flightmare | 高效 UAV RL | 主要依赖 Unity 环境 |
| GS-DroneGym | 3DGS + UAV + dataset tooling | 更接近，但不是 CityNav 式真实数据集 Human Collection |
| **RealSceneUAV** | **真实场景适配 + 连续人类控制 + 可替换动力学 + 数据采集** | — |

其中 **GS-DroneGym** 与我们的目标最接近。后续 3DGS 等模块优先考虑复用成熟方案，而不是重复实现。

## 9. Roadmap

### M2：CityNav / SensatUrban 真场景

- [ ] CityRefer object / description 加载
- [x] CityNav episode 精确复现
- [x] SensatUrban RGB + Height GeoTIFF 接入
- [x] 官方 block ground-level 查询
- [x] Human trajectory pose 统一与 replay 基础
- [x] Top-down trajectory 离线验证
- [ ] 第一视角 RGB / Depth live viewer
- [ ] OSM landmark layer

### M3：真实场景 Renderer

- [ ] Point-cloud / raster RGB-D
- [ ] 3DGS
- [ ] 真实几何碰撞
- [ ] visibility query

### M4：飞行动力学

- [ ] PX4 SITL
- [ ] MAVLink manual control
- [ ] RotorPy
- [ ] UAV vehicle profile
- [ ] Physics / Control / Camera 多频率时钟

### M5：数据集与 Benchmark

- [ ] Seen / Unseen manifest
- [ ] Target / Start 自动采样
- [ ] Human session metadata
- [ ] Dataset validator
- [ ] Replay tool
- [ ] Parquet / LeRobot 导出

## 10. 可复现原则

每次正式数据采集至少记录：

- Git commit；
- 数据集与场景来源；
- 坐标系和单位；
- SceneAdapter 版本；
- Target manifest；
- UAV dynamics backend；
- UAV 参数；
- 手柄映射；
- 随机种子；
- control / physics / camera rate。

必须始终区分：

```text
真实场景数据来源
        !=
无人机动力学来源
```

即使场景来自真实城市重建，也不能因此宣称当前参考动力学等同于真实无人机。

## License

代码采用 MIT License。

CityNav、SensatUrban、PX4 以及其他第三方数据和组件仍遵循其各自许可证。
