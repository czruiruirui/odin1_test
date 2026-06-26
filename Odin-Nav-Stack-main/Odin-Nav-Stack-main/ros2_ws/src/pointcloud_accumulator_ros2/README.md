# Realtime Point Cloud Accumulator (ROS2 Humble)

ROS2 Humble 实时点云累积节点，专为 Odin1 空间感知模块设计。启动后自动累积点云数据，支持在线体素下采样，实时在 RViz2 中显示完整的累积点云。

---

## 功能特性

- **启动即自动累积** — 无需手动 `start/stop`，节点启动后自动开始接收并累积点云
- **实时 RViz2 可视化** — 以可配置频率（默认 2Hz）发布 `/accumulated_cloud`，在 RViz2 中实时显示增长中的地图
- **在线体素下采样** — 通过体素网格累积，有效控制内存占用，默认 0.05m 体素
- **TF 自动变换** — 自动将 `cloud_slam`（`odom` 坐标系）通过 TF 转换到 `map` 坐标系
- **RGB 颜色保留** — 完整保留 Odin1 彩色点云信息
- **服务控制** — 提供 `pause` / `resume` / `clear` / `save` 四个服务接口
- **体素 + 统计双滤波** — 保存时支持体素滤波和统计离群点滤波，生成干净的 PCD

---

## 依赖

- ROS2 Humble
- `rclpy`, `sensor_msgs`, `std_srvs`, `tf2_ros`, `geometry_msgs`
- `sensor_msgs_py` (ROS2 Python 点云工具)
- `python3-numpy`, `python3-scipy`

---

## 编译

```bash
# 进入 ROS2 工作空间
cd ~/Odin-Nav-Stack/ros2_ws  # 或你的 ros2_ws 路径

# 安装缺失依赖（可选）
rosdep install --from-paths src --ignore-src -r -y

# 编译本功能包
colcon build --packages-select pointcloud_accumulator_ros2

# 加载环境
source install/setup.bash
```

---

## 使用步骤

### 1. 启动 Odin1 ROS2 驱动

确保 Odin1 已连接，且 `custom_map_mode` 已设置为建图模式（`1`）：

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

> 驱动启动后，应能看到 `map → odom → odin1_base_link` 的 TF 树。

### 2. 启动实时点云累积器

```bash
# 方式一：使用默认参数
ros2 launch pointcloud_accumulator_ros2 realtime_cloud_accumulator_ros2.launch.py

# 方式二：自定义参数（示例）
ros2 launch pointcloud_accumulator_ros2 realtime_cloud_accumulator_ros2.launch.py \
  cloud_topic:=/odin1/cloud_slam \
  target_frame:=map \
  voxel_size:=0.03 \
  max_points:=5000000 \
  publish_rate:=5.0 \
  auto_start:=true \
  save_rgb:=true
```

节点启动后，日志会输出当前配置和可用服务列表。

### 3. RViz2 中可视化

1. 打开 **RViz2**（可通过 Odin1 驱动自动启动，或手动运行 `rviz2`）
2. 设置 **Fixed Frame** 为 `map`
3. 点击 **Add** → **By Topic** → 选择 **`/accumulated_cloud`** → **PointCloud2**
4. 可选：调整 **Style**（Points / Squares）、**Size** 和 **Color Transformer**（RGB / FlatColor）

随着 Odin1 移动，RViz2 中显示的累积点云会不断增长。

### 4. 服务控制（可选）

在运行过程中，可通过 ROS2 服务接口控制累积器：

```bash
# 暂停累积（保留已累积数据，不再接收新点云）
ros2 service call /realtime_cloud_accumulator/pause std_srvs/srv/Trigger {}

# 继续累积
ros2 service call /realtime_cloud_accumulator/resume std_srvs/srv/Trigger {}

# 清空已累积点云（重置为空白）
ros2 service call /realtime_cloud_accumulator/clear_map std_srvs/srv/Trigger {}

# 保存 PCD 文件
ros2 service call /realtime_cloud_accumulator/save_map \
  pointcloud_accumulator_ros2/srv/SaveMap \
  "filename: 'my_map'
  voxel_size: 0.05
  apply_statistical_filter: true"
```

保存成功后会返回文件路径，默认位于 `ros2_ws/maps/` 目录下。

---

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `cloud_topic` | `string` | `/odin1/cloud_slam` | 输入点云话题（Odin1 驱动默认发布） |
| `target_frame` | `string` | `map` | 累积目标坐标系，需确保 TF 可达 |
| `save_dir` | `string` | `~/maps` | PCD 文件保存目录 |
| `voxel_size` | `float64` | `0.05` | 在线体素下采样大小（米），越大内存占用越小 |
| `max_points` | `int` | `10000000` | 最大累积体素数，达到上限后停止接收 |
| `save_rgb` | `bool` | `true` | 是否保留彩色信息 |
| `pcd_binary` | `bool` | `true` | PCD 保存格式：`true` 为二进制，`false` 为 ASCII |
| `max_stat_points` | `int` | `200000` | 统计滤波点数上限，超过则跳过统计滤波 |
| `publish_rate` | `float64` | `2.0` | `/accumulated_cloud` 发布频率（Hz） |
| `auto_start` | `bool` | `true` | 启动后是否自动开始累积 |
| `max_frames_per_sec` | `float64` | `10.0` | 每秒最多处理帧数，避免 CPU 过载 |

---

## 话题与服务

### 订阅的话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/odin1/cloud_slam` | `sensor_msgs/PointCloud2` | Odin1 驱动发布的彩色 SLAM 点云 |

### 发布的话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/accumulated_cloud` | `sensor_msgs/PointCloud2` | 实时累积的完整点云（用于 RViz2 显示） |

### 提供的服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/realtime_cloud_accumulator/pause` | `std_srvs/srv/Trigger` | 暂停累积 |
| `/realtime_cloud_accumulator/resume` | `std_srvs/srv/Trigger` | 继续累积 |
| `/realtime_cloud_accumulator/clear_map` | `std_srvs/srv/Trigger` | 清空已累积点云 |
| `/realtime_cloud_accumulator/save_map` | `pointcloud_accumulator_ros2/srv/SaveMap` | 保存为 PCD 文件 |

**SaveMap 服务字段：**
- `string filename` — 保存文件名（不含 `.pcd` 后缀会自动补全）
- `float64 voxel_size` — 保存时的体素滤波大小（`0` 表示使用默认 `0.05m`）
- `bool apply_statistical_filter` — 是否应用统计离群点滤波
- 返回：`success`, `message`, `saved_path`, `total_points`, `filtered_points`

---

## 文件结构

```
pointcloud_accumulator_ros2/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── srv/
│   └── SaveMap.srv              # 自定义保存服务
├── launch/
│   └── realtime_cloud_accumulator_ros2.launch.py  # ROS2 Launch 文件
├── pointcloud_accumulator_ros2/
│   ├── __init__.py
│   └── realtime_cloud_accumulator_ros2.py          # 主节点
└── resource/
    └── pointcloud_accumulator_ros2
```

---

## 注意事项

1. **TF 树要求**
   - 节点会自动通过 TF 将 `cloud_slam` 的 `frame_id`（Odin1 驱动中为 `odom`）转换到 `target_frame`（默认为 `map`）。
   - 确保 Odin1 驱动已发布 `map → odom` 的 TF 变换，否则累积器会提示 `TF 查询失败`。
   - 可通过 `ros2 run tf2_tools view_frames` 查看 TF 树是否完整。

2. **QoS 匹配**
   - Odin1 ROS2 驱动发布的点云使用 `BEST_EFFORT` QoS。本节点订阅者也配置为 `BEST_EFFORT`，确保数据能正确接收。
   - 如果修改了驱动的 QoS，请确保与本节点一致。

3. **内存控制**
   - 默认 `max_points` 为 `10000000`（一千万体素），以 `0.05m` 体素为例，约覆盖 `500m × 500m × 10m` 空间。
   - 如果场景很大，可适当增大 `voxel_size`（如 `0.10`）以降低内存占用。

4. **保存路径**
   - 默认保存到 `ros2_ws/maps/`。如果目录不存在，节点会自动创建。
   - 可通过 `save_dir` 参数指定其他路径，如 `/home/user/my_maps`。

5. **RGB 颜色**
   - Odin1 驱动发布的 `cloud_slam` 包含 RGB 信息（编码为 `float32` 的 `rgb` 字段）。
   - 如果不需要颜色，设置 `save_rgb:=false` 可显著降低内存和 PCD 文件大小。

---

## 常见问题

### Q1: RViz2 中没有显示 `/accumulated_cloud`？

- 检查 **Fixed Frame** 是否设置为 `map`（或你的 `target_frame`）。
- 检查 TF 树是否完整：`ros2 run tf2_tools view_frames`。
- 检查点云话题是否有数据：`ros2 topic hz /odin1/cloud_slam`。
- 检查节点日志是否有 `TF 查询失败` 或 `已达到最大点数限制`。

### Q2: 保存的 PCD 文件在哪里？

- 默认路径：`~/Odin-Nav-Stack/ros2_ws/maps/`（或你编译时 ros2_ws 的同级目录）。
- 也可通过服务返回的 `saved_path` 字段确认。

### Q3: 如何只保存部分区域的点云？

- 目前版本通过 `pause` 服务暂停累积后，再 `save_map` 即可保存当前累积区域。
- 后续可通过 RViz2 的 **Publish Point** 或自定义 ROI 接口实现区域选择。

---

## 许可

Apache License 2.0

---

*Maintainer: Manifold Tech Ltd.*
*For Odin1 Navigation Stack — ROS2 Humble Edition*
