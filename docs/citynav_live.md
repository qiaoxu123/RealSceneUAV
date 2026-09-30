# CityNav 实时飞行与 Human Trajectory 采集

M2b 在 M2a 的 CityNav / SensatUrban raster 基础上增加实时可视化和 Switch 人类控制。

## 1. 当前界面

Live Viewer 包含三栏：

```text
+----------------+----------------+----------------+
|      RGB       |     Depth      |    Top-down    |
|                |                |                |
| 当前真实场景   | 当前深度       | 正射真实地图   |
| observation    | observation    | + 实时轨迹     |
+----------------+----------------+----------------+
| Instruction / XYZ / Yaw / Target Distance / Status |
+-----------------------------------------------------+
```

Top-down 中显示：

- 当前 UAV 位置；
- 当前航向；
- 已飞轨迹；
- target 位置。

## 2. 安装

```bash
pip install -e ".[controller,citynav]"
```

先确认 Switch / 手柄映射：

```bash
realsceneuav-controller-check --seconds 15
```

## 3. 从一条 CityNav episode 开始飞

```bash
realsceneuav-citynav-collect \
  --trajectory-json data/citynav_train_seen.json \
  --episode-index 0 \
  --rgb-png data/rgbd/cambridge_block_2.png \
  --height-tif data/rgbd/cambridge_block_2.tif \
  --controller gamepad \
  --controller-config configs/switch_pro.yaml \
  --control-hz 50 \
  --camera-hz 10 \
  --duration 300 \
  --output outputs
```

该命令会：

1. 从 released CityNav trajectory 中读取原始 start pose；
2. 读取该 episode 的 language instruction；
3. 读取真实 target position；
4. 加载对应 SensatUrban / CityNav RGB + Height raster；
5. 在真实场景数据上实时生成 RGB / Depth；
6. 使用 Switch 连续控制 UAV；
7. 显示实时 top-down trajectory；
8. 同步记录 Human Control、UAV State、RGB、Depth 和事件。

## 4. 多次采同一任务

同一个 source episode 可以重复采集。

默认会自动附加 UTC run ID，例如：

```text
citynav-cambridge_block_2-000123-20260930T021500Z
```

也可以手动指定：

```bash
--run-id pilot01
```

这样便于：

- 多个操作者采同一任务；
- 同一个人重复试飞；
- 比较不同控制设置；
- 构建 Human Demonstration 数据集。

## 5. 无手柄调试

可以先使用 scripted controller 验证数据链：

```bash
realsceneuav-citynav-collect \
  --trajectory-json data/citynav_train_seen.json \
  --episode-index 0 \
  --rgb-png data/rgbd/cambridge_block_2.png \
  --height-tif data/rgbd/cambridge_block_2.tif \
  --controller scripted \
  --no-viewer \
  --no-realtime \
  --duration 2
```

## 6. 数据输出

每次采集仍使用统一 episode schema：

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

其中 metadata 会额外记录：

- source CityNav JSON；
- source episode index；
- map name；
- raster source；
- ground level；
- controller mapping；
- control / camera rate；
- dynamics backend。

## 7. 当前动力学说明

当前 CityNav live collection 默认仍使用：

```text
ReferenceQuadrotorDynamics
```

它用于先验证：

- 数据集；
- 坐标；
- UI；
- Switch；
- Recorder；

整条链是否正确。

**当前版本不能把该 reference dynamics 宣称为真实无人机的经过验证动力学。**

下一阶段 M3/PX4 会保持完全相同的 CityNav / Viewer / Recorder 接口，仅替换：

```text
ReferenceQuadrotorDynamics
            |
            v
PX4 SITL / MAVLink backend
```

这样场景数据层和飞行动力学层始终独立。
