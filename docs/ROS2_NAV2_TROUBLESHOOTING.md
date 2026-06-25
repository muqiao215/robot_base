# ROS2 + Nav2 排障手册（麦轮机器人实战版）

> 基于本项目真机联调（树莓派 ROS2 Jazzy + Nav2 + AMCL + RPP + SmacPlanner2D）踩坑总结。
> 用法：**先按"症状"定位，再跑"诊断命令"取证，最后按"修复"动手。不要跳过取证直接改参数。**

---

## 0. 每条 SSH 命令的环境前缀（Pi 上跑 ROS2）

```bash
ssh ubuntu@100.117.38.82          # Tailscale 免密；备用局域网 172.20.10.2
export LC_ALL=C; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0
```

- `colcon symlink-install`：install→build→**src** 是符号链接，改 src 的 `.py`/`.yaml` **直接生效**（Python 节点要重启，launch 新文件要 rebuild）。
- ⚠️ RViz 在**另一台 24.04 机器**上（和 Pi 已通 DDS）。**别在装 docker 的本机开 RViz**（docker0 网卡 + 跨网段，DDS 发现不到 Pi）。
- ⚠️ SSH 在「kill 大量进程」时常**无回传**，需另发一条 `ps` 命令确认。

---

## 1. 黄金诊断流程（任何导航问题先跑这套）

```bash
# (1) 进程在不在
ps -ef | grep -E "drive_node|component_container|nav_all" | grep -v grep

# (2) Nav2 生命周期是否全 active（不是 active 就是没起来）
for n in /controller_server /planner_server /amcl /map_server \
         /global_costmap/global_costmap /local_costmap/local_costmap; do
  echo -n "$n: "; ros2 lifecycle get $n; done

# (3) 定位对不对（AMCL 在角落 = 没重定位）
ros2 topic echo /amcl_pose --once --field pose.pose.position

# (4) TF 链是否完整 + 实时
ros2 run tf2_ros tf2_echo map base_link | head -6

# (5) 传感器活着吗
ros2 topic hz /scan        # 应 ~10Hz

# (6) 看当前导航日志（启动时写进 /tmp/nav2_current_log）
tail -40 "$(cat /tmp/nav2_current_log)"
```

**生命周期状态码**：`unconfigured[1]` 没配置 → `inactive[2]` 没激活 → `active[3]` 正常。
任何一个卡在 1 或 2，导航就**静默失败**。

---

## 2. 症状速查表

### 症状 A：`GridBased plugin failed to plan ... "no valid path found"`，且**瞬间**失败（~10ms）

**别先怀疑地图。** 瞬间失败 = planner 在搜索前就把起点/终点判死了。

**诊断**（直接查那两个点的实际 cost）：
```bash
# 起点/终点 cost=0 表示物理上是自由空间，问题在 planner 内部
ros2 service call /global_costmap/get_cost_global_costmap nav2_msgs/srv/GetCost \
  "{x: <起点x>, y: <起点y>, use_footprint: false}"
# 直接发规划 action（不驱动机器人，零风险）验证能否出路径
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: <gx>, y: <gy>}, orientation: {w: 1.0}}}, use_start: false}"
```

**最常见根因 + 修复**：

| 根因 | 现象 | 修复 |
|---|---|---|
| **`downsample_costmap: True`**（SmacPlanner2D） | get_cost=0 但仍失败；0.1m 图被压成 0.2m 粗网格，取 2×2 最大 cost，贴墙自由格被"传染"成 lethal | `downsample_costmap: False` |
| 起/终点真在 lethal 区 | get_cost ≥ 253 | 换目标点 / 减小 inflation_radius（但别低于内切半径，见症状 C） |
| `track_unknown_space: True` + 地图有 unknown | planner 把未知当障碍 | `allow_unknown: True` 或重建地图 |

> 💡 关键工具：`get_cost` 查的是**全分辨率原图**，planner 跑的可能是**降采样图**——两者不一致正是"点是空的却 no path"的指纹。

---

### 症状 B：`RPP detected collision ahead!` 反复触发 → spin 恢复 → 又碰撞 → 原地转圈/Goal failed

**先分清"前方碰撞"是真障碍还是幻影**：看 RViz 激光点云。
- 点云**糊开/拖尾**、且障碍出现在机器人**正在转的方向** → **幻影**（TF 滞后污染 costmap）
- 点云贴合墙、前方物理上确有东西 → 真障碍（见症状 C/D）

**幻影根因链（本项目实测）**：
```
Pi 算力卡 → 激光 TF 时间戳滞后 → 机器人旋转时每帧扫描贴到"过时朝向"
  → 墙被抹成扇形云，糊进门洞 → costmap 冒幻影墙 → RPP 碰撞 → spin → 一转又糊更多 → 死循环
```

**致命配置 bug —— marking 范围 > raytrace 范围**：
```bash
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.obstacle_max_range  # 标记
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.raytrace_max_range  # 清除
```
若 `obstacle_max_range > raytrace_max_range`：**标记的远障碍永远清不掉**（清除射线够不到）→ 幻影永久驻留。

**修复**：
```yaml
obstacle_max_range: 3.0    # 标记必须 ≤ 清除
raytrace_max_range: 3.5    # 清除 ≥ 标记，凡标记的都能被清掉；收近场减少旋转拖尾污染
```
应急清除卡住的幻影：
```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

> ⚠️ 治本是解决 TF 滞后（降 `controller_frequency` 匹配 Pi 算力、提高 `transform_tolerance`、减小激光处理负载）。本项目算力受限，先用收窄 marking 范围缓解。

---

### 症状 C：机器人撞门框 / 撞墙（碰撞检测贴脸才报警）

**根因：机器人尺寸模型没配，Nav2 默认 0.1m（10cm）"幽灵小车"。**

**诊断**：
```bash
ros2 param get /global_costmap/global_costmap robot_radius        # 没配会是默认 0.1
ros2 param get /local_costmap/local_costmap footprint             # 空 = 默认 0.1m 圆
```
真车若约 0.3m 半径却按 0.1m 规划 → 规划出真车过不去的缝、碰撞检测用 10cm footprint 贴脸才报 → 撞。

**修复**（用真实尺寸）：
```yaml
global_costmap:
  robot_radius: 0.30                  # 内切半径
local_costmap:
  footprint: "[[0.25,0.26],[0.25,-0.26],[-0.25,-0.26],[-0.25,0.26]]"  # 真实多边形 0.5×0.52m
```

---

### 症状 D：`Inflation layer ... not set sufficiently` + 设小 inflation 后反而撞墙

**铁律：`inflation_radius` 必须 ≥ 内切半径（robot_radius / footprint 内切）。**

把 inflation 降到比机器人半径还小，会在"墙边一圈"里把**车身会碰到的格子标成低代价可走** → planner 把车体中心规划进这条带 → 撞。

```bash
ros2 param get /global_costmap/global_costmap robot_radius              # 内切
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
# 必须 inflation_radius >= robot_radius
```

**门难过 ≠ 该把 inflation 压到内切半径以下。** 正确做法：
- `inflation_radius` = 内切半径 + 小缓冲（如内切 0.30 → inflation 0.35）
- 靠 `cost_scaling_factor`（调大=代价衰减快=敢贴墙但不撞）让窄门可过

本项目最终值：global inflation 0.35（内切0.30）、local inflation 0.30（内切0.25）、cost_scaling 3.0。

---

### 症状 E：重启 nav_all 后机器人定位跑到地图角落

`set_initial_pose: true` + `initial_pose` 指向地图角落 → 每次重启 AMCL 回到角落，**定位是错的**。

**修复**：重启后必须在 RViz 点 **2D Pose Estimate** 重定位（位置差 20cm 没事，**朝向差 10° 激光点云就全错开**，反复微调箭头直到点云"咬"住墙）。
或命令行设位姿：
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: <x>, y: <y>}, orientation: {w: 1.0}}}}"
```

---

### 症状 F：`Control loop missed its desired rate of 10Hz. Current 5.5Hz`

Pi 算力不足，控制环跑不到目标频率 → 反应迟钝、碰撞检测晚、放大顿挫。

**缓解**：`controller_frequency: 10.0 → 8.0`（匹配 Pi 实际算力，消除 missed-rate 告警，减少 TF 滞后的连锁污染）。属优化项，非阻塞。

---

## 3. 安全 & 操作纪律（真机移动时）

- **机器人会动前，每次先确认场地安全、人离开、手边能急停。**
- 急停：`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`
- 纯规划测试（不驱动）：用 `/compute_path_to_pose` action，零风险验证 planner。
- **架构分离**：底盘 `drive.launch.py`（drive_node + laser TF）和导航 `nav_all.launch.py`（Nav2）分开。调底盘只重启前者，调导航只重启后者。**重启导航时确认 drive_node 存活。**

---

## 4. 配置漂移——本项目最大的隐形时间杀手

**症状**：热改 `ros2 param set` 立刻见效，但**一重启全部回退**，反复"修好又坏"。

**根因**：三处配置不同步——① 桌面 git 仓库；② Pi 的 `src/.../nav2_params.yaml`（重启读这个）；③ Pi 运行时 live 值（热改只改这个）。

**纪律**：
1. 热改只用于**快速验证**，验证通过后**立刻写回 yaml**（桌面仓库 + scp 到 Pi）。
2. 改大量参数时，**以桌面仓库 yaml 为单一真相源**，改完整文件后 `scp` 覆盖 Pi，一次性消除漂移。
   ```bash
   scp robot_base/config/nav2_params.yaml ubuntu@<pi>:~/ros2_ws/src/robot_base/config/nav2_params.yaml
   ```
3. scp 前先在 Pi 上备份：`cp nav2_params.yaml nav2_params.yaml.bak.<原因>_<时间>`
4. 改 `param set` 时注意：**costmap 的某些参数（如 inflation/obstacle range）支持动态重配，但 planner 插件参数（downsample_costmap 等）可能在 configure() 时才读**——热改不一定立刻重建插件，必要时重启该节点验证。

---

## 5. 踩过的坑（别重来）

- ❌ 别同时跑脚本读 `ttyUSB0`：drive_node 独占串口，双开崩（"multiple access on port"）。
- ❌ 别用 `ros2 topic echo --once` 轮询 odom 做实时控制：太慢（几百 ms/次），机器人会转过头。
- ❌ 别写 Fast-DDS 单播 XML（`~/fastdds_tailscale.xml`）：破坏本地节点间通信，odom/TF 全断。
- ❌ 改 yaml 正则替换时**限定 count 和区段**：local_costmap 在文件里可能排在 global 前面，无限制的 `re.sub` 会误伤 local（曾导致 `local_costmap: No 'plugin' param` 启动崩溃）。改 costmap 参数按区段解析，别全局正则。
- ❌ 短窗口（<2.5s）采样标定不可信：加减速过程污染稳态，要跑够时间只取稳态段。

---

## 6. 关键参数最终值（本项目，2026-06-25）

| 参数 | 值 | 理由 |
|---|---|---|
| `planner GridBased.downsample_costmap` | False | 避免粗网格把贴墙自由格判 lethal |
| `local obstacle_max_range` / `raytrace_max_range` | 3.0 / 3.5 | marking ≤ raytrace，幻影可清除 |
| `global robot_radius` | 0.30 | 真车内切半径，非默认 0.1 |
| `local footprint` | 0.5×0.52m 多边形 | 真车尺寸 |
| `global inflation_radius` | 0.35 | ≥ 内切 0.30 + 缓冲 |
| `local inflation_radius` | 0.30 | ≥ 内切 0.25 + 缓冲 |
| `cost_scaling_factor` (both) | 3.0 | 代价衰减快，窄门可过 |
| `RPP use_cost_regulated_linear_velocity_scaling` | true | 窄处主动减速，不再冲-撞-停 |
| `RPP max_allowed_time_to_collision_up_to_carrot` | 0.6 | 收紧碰撞前瞻 |

---

## 7. 待办 / 已知非阻塞项

- TF 时间戳滞后（旋转时拖尾）治本需解决 Pi 算力：考虑降 `controller_frequency` 10→8、激光降采样、或更强算力板。
- AMCL 在地图最角落（x≈-73）偶发 `Sensor origin out of map bounds` 告警，地图边界问题，先观察。
- `/map_rviz` 偶有多发布者（重启残留 map_republisher 孤儿进程），清理：`pkill -f map_republisher` 后重启 nav_all。
</content>
</invoke>
