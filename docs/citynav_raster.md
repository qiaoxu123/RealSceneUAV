# CityNav / SensatUrban 真场景接入

本模块用于验证 RealSceneUAV 与 CityNav 官方数据坐标和视觉生成逻辑是否一致。

## 1. 数据关系

CityNav 的 released Human Trajectory JSON 提供：

- area / block
- language description
- target position
- marker position
- human trajectory
- split

SensatUrban / CityNav 的 raster 数据提供：

- 正射 RGB PNG
- 表面高度 GeoTIFF

因此一条真实场景 observation 的生成过程是：

```text
CityNav Human Pose
(x, y, z, yaw)
        |
        v
官方 block ground level
        |
        v
计算当前 UAV square view footprint
        |
        v
映射到正射 RGB / Height raster
        |
        v
Perspective Crop
        |
  +-----+------+
  |            |
 RGB       Surface Height
               |
               v
        Depth = UAV_z - height
```

这与 CityNav 官方 `cropclient.py` 的基本逻辑保持一致。

## 2. 安装

```bash
pip install -e ".[citynav]"
```

## 3. 验证一条 Human Trajectory

假设：

```text
data/
├── citynav_train_seen.json
└── rgbd/
    ├── cambridge_block_2.png
    └── cambridge_block_2.tif
```

执行：

```bash
realsceneuav-citynav-validate \
  --trajectory-json data/citynav_train_seen.json \
  --episode-index 0 \
  --rgb-png data/rgbd/cambridge_block_2.png \
  --height-tif data/rgbd/cambridge_block_2.tif \
  --output outputs/citynav_episode_0
```

如果不手动提供 `--ground-level`，程序会优先使用 CityNav 官方发布的 block ground level。

## 4. 输出

```text
outputs/citynav_episode_0/
├── summary.json
├── trajectory_topdown.png
├── rgb_000000.png
├── depth_000000.npy
├── rgb_<middle>.png
├── depth_<middle>.npy
├── rgb_<last>.png
└── depth_<last>.npy
```

其中：

### trajectory_topdown.png

在真实正射 RGB 上绘制：

- Human trajectory
- start point
- target point

用于首先检查：

```text
trajectory coordinate
        =
target coordinate
        =
raster coordinate
```

如果这里对不齐，就不能继续做 live viewer 或模型训练。

### RGB / Depth

分别保存 Human trajectory 的：

- 起点
- 中间点
- 终点

三个第一视角/俯视 observation，用于检查：

- yaw 是否正确；
- 飞行高度是否正确；
- RGB crop 是否与轨迹位置一致；
- Depth 是否与真实表面高度一致。

## 5. Pose 兼容

CityNav 历史数据存在两种 pose 表示：

```text
[x, y, z, yaw, pitch]
```

以及：

```text
[x, y, z, dx, dy, dz]
```

RealSceneUAV 会统一转换成：

```text
[x, y, z, yaw, pitch]
```

不会在下游代码中混用两种格式。

## 6. 为什么先做 Raster，而不是直接做 3DGS？

因为当前最重要的是先验证：

1. CityNav trajectory 坐标；
2. SensatUrban raster 坐标；
3. target 坐标；
4. UAV heading；
5. altitude / depth；

是否完全一致。

Raster 路径是 CityNav 官方公开训练/评测代码实际使用的路径之一，因此最适合做第一阶段基准。

在这一步确认以后，M2b 再加入：

- 实时 first-person viewer；
- top-down live trajectory；
- instruction / target / landmark overlay；
- Switch 实时飞行。

3DGS 则作为后续可替换 renderer 接入，而不是改变 CityNav 数据层。
