# PX4 SITL / MAVLink 后端

RealSceneUAV 将 PX4 作为可替换 `DynamicsBackend` 接入，同时保持 CityNav 场景、Viewer、Switch 和 Recorder 不变。

## 1. 当前支持

- MAVLink heartbeat；
- `MANUAL_CONTROL` 连续输入；
- `LOCAL_POSITION_NED` 与 `ATTITUDE` 遥测；
- PX4 NED/FRD 到 RealSceneUAV z-up 世界坐标转换；
- CityNav 起始位置与起始 yaw 对齐；
- telemetry timeout；
- PX4 mode / arm / disarm；
- episode metadata 记录 PX4 配置与坐标映射。

## 2. 安装

```bash
pip install -e ".[controller,citynav,px4]"
```

## 3. 启动 PX4 SITL

当前 PX4 官方 Gazebo X500 启动方式：

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

PX4 默认常用 UDP 端口：

```text
14550 -> GCS / QGroundControl
14540 -> Offboard API
```

RealSceneUAV 默认使用 `udpin:0.0.0.0:14540`，便于与 QGroundControl 共存。

## 4. 手动控制

RealSceneUAV 使用 MAVLink `MANUAL_CONTROL`：

```text
pitch    -> x
roll     -> y
throttle -> z
yaw      -> r

x/y/r: [-1000, 1000]
z:     [0, 1000]   # PX4 当前兼容路径
```

建议在 PX4/QGroundControl 中明确选择允许 MAVLink joystick 的 `COM_RC_IN_MODE` 配置。

## 5. CityNav + PX4

```bash
realsceneuav-citynav-collect \
  --trajectory-json data/citynav_train_seen.json \
  --episode-index 0 \
  --rgb-png data/rgbd/cambridge_block_2.png \
  --height-tif data/rgbd/cambridge_block_2.tif \
  --controller gamepad \
  --controller-config configs/switch_pro.yaml \
  --dynamics px4 \
  --px4-connection udpin:0.0.0.0:14540 \
  --px4-mode POSCTL \
  --px4-arm \
  --control-hz 50 \
  --camera-hz 10
```

PX4 manual control 需要持续输入，因此 PX4 模式要求 `control_hz >= 10`，默认仍为 50 Hz。

## 6. 坐标系

```text
PX4 World: NED       -> RealSceneUAV: x=east, y=north, z=up
PX4 Body : FRD       -> RealSceneUAV Body: FLU
```

每条 episode 开始时，当前 PX4 local frame 会锚定到 CityNav 的 source start position，并通过 yaw offset 对齐 source start yaw。

## 7. Reset

`ReferenceQuadrotorDynamics` 可以瞬移 reset；PX4 不允许伪造该行为。PX4 模式下按 Reset 会记录 `reset_unsupported`，不会让视觉坐标假装回到起点。

## 8. 当前边界

当前结构是：

```text
真实 CityNav/SensatUrban visual scene
                +
PX4/Gazebo flight controller + dynamics
```

但目前：

```text
Gazebo collision geometry != CityNav real-city geometry
```

因此 PX4 当前提供真实飞控逻辑和动力学响应，而 CityNav 提供真实视觉/地图/目标。后续还需要把真实点云、mesh 或其他真实几何接入物理碰撞层。

## 9. 安全边界

当前阶段优先使用 SITL。进入 HIL / Pixhawk / 真实无人机前，必须重新验证 vehicle mass、inertia、thrust、failsafe、telemetry loss、emergency stop 和 geofence。

## 10. 官方参考

- https://docs.px4.io/main/en/config/manual_control
- https://docs.px4.io/main/en/config/joystick
- https://docs.px4.io/main/en/simulation/
- https://mavlink.io/en/services/manual_control.html
