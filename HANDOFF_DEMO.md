# HANDOFF — 演示流程（玻璃反射环境）2026-06-25

> 接手就照这文件走。核心结论：**玻璃反射无法用配置根治，必须用流程兜底**——录点位 + 每段重定位 + 短点列。

---

## 0. 一句话现状

底盘/导航参数全部调通（详见 `HANDOFF_CHASSIS_CAL.md` + `ROS2_NAV2_TROUBLESHOOTING.md`）。
**演示的真正难点是玻璃反射**：走廊门洞的玻璃幕墙会把激光反射回传感器，导致 ±5cm 漂移，无法靠参数过滤。
解决思路：footprint 已 +5cm 加宽吸收漂移；**演示靠录好的固定点位 + 每段重定位绕开问题区**。

---

## 1. 演示前的标准流程（必做清单）

### 1.1 系统健康检查（30 秒）

```bash
ssh ubuntu@100.117.38.82          # Tailscale 免密
export LC_ALL=C; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0

# (1) 进程在不在
ps -ef | grep -E "drive_node|component_container|nav_all" | grep -v grep

# (2) 导航栈是否全 active（任何一个不是 active 就是没起来）
for n in /controller_server /planner_server /amcl \
         /global_costmap/global_costmap /local_costmap/local_costmap; do
  echo -n "$n: "; ros2 lifecycle get $n; done

# (3) 关键参数已加载（footprint 是 +0.05m 版）
ros2 param get /local_costmap/local_costmap footprint
# 期望: [[0.30, 0.31], [0.30, -0.31], [-0.30, -0.31], [-0.30, 0.31]]

ros2 param get /global_costmap/global_costmap robot_radius
# 期望: 0.31
```

### 1.2 启动（按顺序）

```bash
# 终端 1：底盘
ros2 launch robot_base drive.launch.py

# 终端 2：导航（拆出去的架构，只跑 Nav2 + map_republisher）
ros2 launch robot_base nav_all.launch.py
```

### 1.3 **必须** 2D Pose Estimate 精确定位

> **每场演示前必做。** 重启后 AMCL 默认落在地图角落 (-73.486, -4.608)，定位是错的。
>
> **位置差 20cm 没事，朝向差 10° 激光点云就全错开。** 所以反复微调朝向是关键。

操作步骤：
1. RViz Fixed Frame = `map`，加显示：Map (`/map` 或 `/map_rviz`)、LaserScan (`/scan`)、TF、Costmap
2. 工具栏点 **2D Pose Estimate**
3. **位置**：放到机器人**真实所在**的 map 坐标上（看周边墙线做参考）
4. **朝向**：箭头方向对准机器人**车头真实朝向**——重点是激光点云**贴合墙壁**，不贴合就原地再点一次微调角度
5. 验证：`ros2 topic echo /amcl_pose --once --field pose.pose.position` 看是不是你点的位置

**关键判据**：点完后 RViz 里**红绿激光点云必须紧贴紫色地图的墙线**，错开就再调。

---

## 2. 关于"点位"——本演示的核心概念

### 2.1 为什么必须录点位

- 玻璃反射造成 ±5cm 漂移，让车"以为"自己在某个位置其实偏了
- 端到端远距离一键导航：必经玻璃区 → 漂移累积 → 卡住或撞
- **短点列 + 已知好点**：每段路径已知成功过，演示稳定可重复

### 2.2 为什么要"录制点位"

**客观原因（重要，请理解）**：

演示场地的地图坐标系是建图时定的，世界坐标 (map frame) 是固定参考系。但：

1. **每次开机、每次重定位**，AMCL 给出的"地图坐标"是**机器人当前帧的位姿估计**，它会受激光漂移影响——同一物理位置，多次重定位可能给到不同的 (x, y, yaw) 三元组
2. **演示当天**机器人的起点、起始 yaw 由你现场 2D Pose Estimate 决定，**不是脚本预定的固定值**
3. **map 坐标 (x, y, yaw) 取决于建图原点的选择**——除非你的地图原点是某个固定物（门框、墙角），否则坐标值本身**没有绝对物理意义**

**所以"录制点位"不是"把坐标写到脚本里"，而是：**

- 在**当次演示**重定位完成后，**驾驶机器人（或手动推到）每个关键位置**，**当场读出当时的 AMCL 坐标**（见 §2.3）
- 把这些**相对当前重定位的坐标值**记下来，作为**当天演示的 waypoint 列表**
- 每段目标都从这张表里取，**不要从建图原点的绝对坐标取**——因为你已经重定位了，坐标系原点可能差几十厘米

### 2.3 如何录点位（当天）

走到目标点 → 让车停稳 → 执行：

```bash
ros2 topic echo /amcl_pose --once --field pose.pose
```

输出格式：
```
position: {x: <X>, y: <Y>, z: 0.0}
orientation: {x: 0, y: 0, z: <qz>, w: <qw>}
# yaw = atan2(2*qz*qw, 1 - 2*qz*qz)
```

记录到下面 §3 的清单里。

**更省事的办法**：在 RViz 里**用 2D Pose Estimate（注意不是 Goal Pose）点击机器人当前位置**，从 nav2 日志里读 `amcl: Setting pose (时间戳): <x> <y> <yaw>` 这一行。

---

## 3. 演示点位记录模板（按你场地填）

> **每次演示前现场录制。** 不要复用上次录的——重定位坐标会变。
> 录制时机：定位完成后，驾驶/推车依次到每个关键点，停留 2 秒读坐标。

### 3.1 走廊入口 → 门口 → 走廊出口 模板

```yaml
demo_session:
  date: 2026-06-25
  relocalized_at: "(重定位后填 yyyy-mm-dd HH:MM)"
  amcl_initial_after_reloc: "(填 2D Pose Estimate 后第一次 amcl_pose 的 x/y/yaw)"

waypoints:
  # 起步段：从演示起点到门口前
  - idx: 1
    x: 0.0          # ← 现场读到的 AMCL x
    y: 0.0          # ← 现场读到的 AMCL y
    yaw_deg: 0.0    # ← atan2(2*qz*qw, 1-2*qz*qz) 换算
    distance_to_next: 1.2   # 单位 m
    note: "演示起点。空地，激光无反射。"
    checkpoint: "发目标前确认车头正对下一段中线"

  - idx: 2
    x: 1.2
    y: 0.0
    yaw_deg: 0.0
    distance_to_next: 0.5
    note: "门前 0.5m。第一次接近玻璃。"
    checkpoint: "看激光点云——若开始糊开，先停下来 2D Pose Estimate 重定位再继续"

  - idx: 3
    x: 1.7
    y: 0.0
    yaw_deg: 0.0
    distance_to_next: 0.3
    note: "门框前 0.3m，进入玻璃反射核心区。"
    checkpoint: "RPP 应该主动减速（cost_regulated）。若全速冲，停车"

  # 门口段：必须一气呵成，不要中途停
  - idx: 4
    x: 2.0
    y: 0.0
    yaw_deg: 0.0
    distance_to_next: 0.3
    note: "门洞中线。"
    checkpoint: "玻璃反射最强位置。不要在这里停，让车冲过去"

  - idx: 5
    x: 2.3
    y: 0.0
    yaw_deg: 0.0
    distance_to_next: 0.5
    note: "门后 0.3m，已脱离玻璃核心区。"
    checkpoint: "若车偏离中线，立刻 2D Pose Estimate 拉回"

  # 出门段
  - idx: 6
    x: 2.8
    y: 0.0
    yaw_deg: 0.0
    distance_to_next: 1.2
    note: "门口后 0.8m。走廊空地。"
    checkpoint: "可在这里喘口气观察"

  - idx: 7
    x: 4.0
    y: 0.0
    yaw_deg: 0.0
    distance_to_next: null
    note: "演示终点。"
    checkpoint: "确认车到达，激光点云仍贴墙"
```

### 3.2 如何使用这张表

**演示过程中，按 idx 顺序发目标**：

```bash
# 通用命令（每个点改 x/y/yaw）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map},
           pose: {position: {x: <x>, y: <y>, z: 0.0},
                  orientation: {z: <qz>, w: <qw>}}}}"
```

或者在 RViz 用 **2D Goal Pose** 工具，按 (x, y) 在地图上点击（yaw 用拖动箭头方向）。

**推荐用 RViz 2D Goal Pose**，更直观、能立刻看到规划路径是否合理。

---

## 4. 关键约束与红线

### 4.1 演示中**禁止**做的事

- ❌ **不要从起点一键发到终点**：远距离必经过玻璃区，必出问题
- ❌ **不要在玻璃核心区（门洞中线 ±0.3m）停车**——反射漂移会累积
- ❌ **不要相信 Nav2 的自动 recovery**——它在玻璃区会触发更多旋转，制造更多反射漂移
- ❌ **不要用 waypoint-follower 自动跑全程**——它不会"看"激光漂移

### 4.2 演示中**必须**做的事

- ✅ **每段到达后，确认激光点云仍贴墙**——脱墙就 2D Pose Estimate 重定位
- ✅ **门口前后各 0.5m 一气呵成**——发 idx 3 后**紧跟**发 idx 5/6，不要中间停
- ✅ **手里随时能停车**：`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`
- ✅ **车偏离中线立刻 2D Pose Estimate 拉回**，不等 Nav2 自己纠正

### 4.3 演示中的应急流程

```
发现车开始偏离/打转
    ↓
[1] 立刻停车（ros2 cmd_vel 空指令）
    ↓
[2] 看 RViz 激光点云
    ├─ 仍贴墙 → 重新发 idx 下一个目标（Nav2 状态可能还在）
    └─ 糊开/拖尾 → 进 [3]
    ↓
[3] 2D Pose Estimate 拉回真实位置（看周边墙线做参考）
    ↓
[4] 确认激光点云贴墙 → 重发 idx 下一个目标
    ↓
[5] 若反复 [3]-[4] → 跳过当前 idx 改发再下一个（演示流程 > 逐点到达）
```

---

## 5. 配置文件当前值（出错时核验用）

### 5.1 Pi 端 yaml 关键项（持久化）

| 参数 | 值 | 位置 |
|---|---|---|
| local footprint | `[[0.30, 0.31], [0.30, -0.31], [-0.30, -0.31], [-0.30, 0.31]]` | `local_costmap.footprint` |
| global robot_radius | 0.31 | `global_costmap.robot_radius` |
| local inflation_radius | 0.30 | `local_costmap.inflation_layer.inflation_radius` |
| global inflation_radius | 0.35 | `global_costmap.inflation_layer.inflation_radius` |
| cost_scaling_factor | 3.0 | 两处 inflation_layer |
| downsample_costmap | False | `planner_server.GridBased.downsample_costmap` |
| obstacle_max_range | 3.0 | `local_costmap.obstacle_layer.scan` |
| raytrace_max_range | 3.5 | `local_costmap.obstacle_layer.scan` |
| RPP use_cost_regulated | true | `controller_server.FollowPath.use_cost_regulated_linear_velocity_scaling` |
| RPP max_time_to_collision | 0.6 | `controller_server.FollowPath.max_allowed_time_to_collision_up_to_carrot` |
| bt action_server_result_timeout | 120s | `bt_navigator.action_server_result_timeout` |

### 5.2 Pi 端 yaml 同步命令（如果改了桌面仓库忘了同步）

```bash
# 在桌面仓库根目录
scp robot_base/config/nav2_params.yaml ubuntu@100.117.38.82:~/ros2_ws/src/robot_base/config/nav2_params.yaml

# 然后 Pi 上：
ros2 launch robot_base nav_all.launch.py   # 重启导航使配置生效（不影响 drive_node）
```

---

## 6. 演示当天的检查表（打印出来走）

```
□ ssh ubuntu@100.117.38.82 通
□ drive_node + nav2_container 都在跑
□ Nav2 全 lifecycle active
□ local footprint = [[0.30, 0.31], ...]  ← +5cm 版
□ robot_radius = 0.31
□ 2D Pose Estimate 完成，激光点云贴墙
□ 录好 7 个 waypoint（按 §3 模板）
□ 手里能发停车命令
□ 场地上无障碍物/人
□ 急停命令测试过：ros2 topic pub --once /cmd_vel ... "" 能停

【开始演示】
□ idx 1 → 到达 → 确认
□ idx 2 → 到达 → 确认
□ idx 3 → 到达 → 确认
□ idx 4 → 到达（一气呵成不停留）
□ idx 5 → 到达 → 确认
□ idx 6 → 到达 → 确认
□ idx 7 → 到达 → 完成
```

---

## 7. 已知非阻塞问题

- AMCL 重启后会落到地图角落 (-73.486, -4.608)：靠 §1.3 的 2D Pose Estimate 解决
- TF 时间戳滞后（Pi 算力限制）：已通过收缩 marking 范围缓解，未根治
- `controller_frequency` 实际跑 5.5~6Hz（目标 10Hz）：未根治，可考虑下次降到 8Hz

---

## 8. 文件清单

| 文件 | 内容 |
|---|---|
| `HANDOFF_CHASSIS_CAL.md` | 底盘标定交接 |
| `HANDOFF_DEMO.md` | **本文件（仓库根）：演示流程交接** |
| `docs/ROS2_NAV2_TROUBLESHOOTING.md` | Nav2 排障手册 |
| `docs/DEMO_PLAYBOOK.md` | 演示操作手册（精简版） |
| `robot_base/config/nav2_params.yaml` | 全部参数 |
