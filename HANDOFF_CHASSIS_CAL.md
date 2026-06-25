# HANDOFF — 底盘标定 & 导航联调（2026-06-25）

> 这份文档让新窗口能立刻接手。**先读"当前结论"和"下一步决策点"，再动手。**

---

## 0. 一句话现状

底盘运动学/串口/死区都已修好并验证，**正常速度下运动学是准的**。卡在"低速旋转 odom 放大"，但已查明这是**死区副作用、不是 bug**。**下一步建议：直接测导航**（定位、TF、前进都验证 OK）。

---

## 1. 连接方式（Pi = 树莓派，跑 ROS2 Jazzy）

```bash
ssh ubuntu@100.117.38.82          # Tailscale，免密钥
# 局域网备用 IP: 172.20.10.2（同一热点 172.20.10.x/28）
```

- 用户 `ubuntu`，主机名 `pi`，ROS 2 **jazzy**，`ROS_DOMAIN_ID=0`
- 工作区 `~/ros2_ws`，包 `robot_base`，**colcon symlink-install**（改 src 的 .py/.yaml 直接生效，但 Python 节点要重启；改 launch 新文件要 rebuild）
- 每条 SSH 命令前都要：`export LC_ALL=C; source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0`
- SSH 在「kill 大量进程」时常无回传，需另发一条命令确认进程数
- ⚠️ RViz 在**另一台 24.04 机器**上（和 Pi 已通）；本机（22.04 Humble）因 docker0 网卡 + 跨网段，DDS 发现不到 Pi，**别在本机开 RViz**

---

## 2. 本轮已完成（已部署到 Pi + rebuild，桌面仓库也已改）

| 文件 | 改动 | 状态 |
|---|---|---|
| `setup.py` | **补回 `behavior_trees` 的 data_files**（之前缺这行→自定义 BT XML 从没装进 install→所有 BT 调参白调，是隐藏元凶） | ✅ BT XML 已进 install |
| `drive_node.py` | 运动学 IK/FK 共用 `kinematic_k`；串口健壮性（重连重发 upload、不吞异常、修死循环）；odom 填 twist | ✅ |
| `drive_node.py` | `min_command_pwm` 900→**1150**（电机启动死区，已参数化） | ✅ |
| `nav2_params.yaml` | 合并：**保留 Pi 的 global obstacle_layer**，叠加 `set_initial_pose:true`+`OmniMotionModel`、`movement_time_allowance:10`、`use_astar:True`+限时、`action_server_result_timeout:120`、BT 指向自定义 fast BT | ✅ |
| `drive.launch.py` | **新建**：拆出底盘（drive_node + laser TF），参数化 kinematic_k/feedback_sign/min_command_pwm/fallback | ✅ |
| `nav_all.launch.py` | 移除 drive_node + static_tf，只留 map_republisher + Nav2 | ✅ |

**架构已拆分**（你要求的"drive_node 单独管理"）：
```bash
# 终端1 — 底盘（调试底盘只重启这个）
ros2 launch robot_base drive.launch.py
# 终端2 — 导航
ros2 launch robot_base nav_all.launch.py
```
调底盘免改代码：`ros2 launch robot_base drive.launch.py min_command_pwm:=1200 kinematic_k:=0.31`
热调：`ros2 param set /drive_node kinematic_k 0.31`（kinematic_k/feedback_sign/fallback 支持热调；min_command_pwm 要重启）

Pi 上备份：`drive_node.py.bak.deadband_*`、`*.bak.predeploy_*`、`nav2_params.yaml.bak.predeploy_*`

---

## 3. 当前 drive_node 参数生效值（实测验证过）

```
kinematic_k          = 0.3075   （= (0.445+0.17)/2，FK 已验证准确）
feedback_sign        = -1.0     （已验证：发+vx，odom x 增加，方向对）
min_command_pwm      = 1150     （实测电机落地启动死区，900 只颤不转）
allow_cmd_odom_fallback = true
```

---

## 4. 标定实测结果（关键，别重复踩坑）

**已验证 OK：**
- 定位/TF：RViz 里地图、激光点云重合、机器人位置都对（用户确认）→ `base_link→laser` TF 和 AMCL 都可信
- AMCL 稳定在 `x=-73.486, y=-4.608`，TF 全链 `map→odom→base_link` 完整
- 前进方向正确；死区修复后 vx=0.15 能稳定驱动（PWM=1150，反馈 -210~-235mm/s 四轮均匀）
- **稳态反馈健康**：恒速 vx=0.20 跑 3s，98 帧标准差仅 0.008，**0 丢帧、0 兜底**

**"问题"现象 —— 已查明是死区副作用，非 bug：**
- 线速度：vx=0.20→实际 0.213（**1.065，基本准**）；vx=0.15→偏快（死区把小 PWM 拉到 1150）
- 旋转：wz=0.3→放大 **1.85x**；wz=**0.6→0.97（准！）**
- **结论**：低速时四轮 PWM 都被死区拉到 1150 → 旋转/低速线速度被放大；**速度够大（PWM 自然超死区）就准**。`kinematic_k=0.3075` 本身正确（改它无效，因 IK/FK 都用它会抵消）

**踩过的坑（别重来）：**
- ❌ 别同时跑脚本读 ttyUSB0：drive_node 独占串口，双开会崩（"multiple access on port"）
- ❌ 别用 `ros2 topic echo --once` 轮询 odom 做实时控制：太慢（几百 ms/次）会严重滞后，机器人转过头
- ❌ 短窗口（<2.5s）采样不可信：加减速过程污染稳态。要跑够时间 + 只取 1s 后的稳态段
- ❌ 别写那份 Fast-DDS 单播 XML（`~/fastdds_tailscale.xml`）：会破坏本地节点间通信，导致 odom/TF 全断。已弃用

Pi 上的标定脚本（`/tmp/`，可复用）：`spin_cal.py`（转 5s）、`spin_cal2.py <wz>`（带稳态 wz）、`line_cal.py`、`fb_health.py`（反馈健康度）、`pwm_cal.py`

---

## 5. 下一步决策点（让用户选）

**A.〔推荐〕直接测导航。** 定位/TF/前进都 OK，中速运动学准（RPP `desired_linear_vel:0.18` 正好在准的区间）。低速死区放大只影响精细接近，先看导航整体表现，有问题再针对性调。

**B. 先压低速死区误差。** 比如把死区补偿从"硬拉到 1150"改成更平滑的前馈/分段，或降 `min_command_pwm`（但低速电机会重新转不动）。属于优化，非阻塞。

**C. 继续抠转向标定。** 不建议——已证明正常速度准，低速偏差是死区固有，纯标定改不掉。

---

## 6. 怎么开始测导航（选 A 时）

1. 确认两个 launch 在跑（`ps -ef|grep -E "drive_node|component_container"`），不在就按 §2 启动
2. 让用户在 **24.04 RViz** 上：Fixed Frame=`map`，加 Map(`/map`，收不到改 `/map_rviz`)、LaserScan(`/scan`)、TF、Costmap、Path
3. 用户点 **2D Goal Pose** 发目标（AMCL 已自动初始化，不用先点 2D Pose Estimate）
4. 你从 Pi 侧监控：
   ```bash
   ros2 topic echo /amcl_pose --once          # 定位
   tail -f $(cat /tmp/nav2_current_log) | grep -iE "error|fail|recovery|goal"  # BT/规划
   ros2 action send_goal 也可命令行发目标测试
   ```
5. 重点看：能否规划出路径、机器人是否平顺跟踪、转弯处定位是否漂、失败时 fast BT 是否快速恢复（不再卡 90s）
6. ⚠️ 机器人会移动，确认场地安全；停车：`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`

---

## 7. 已知待办（非阻塞）
- planner 启动有 `inflation layer ... not set sufficiently` 告警 + 偶发 `Sensor origin out of map bounds`（地图边界，定位在地图最角落 -73.486）。先观察，影响规划再调 costmap inflation
- `/map_rviz` 有时多发布者（重启残留 map_republisher），不影响
- 桌面仓库改动尚未 commit（用户没要求）；Pi src 不是 git 仓库（手动拷贝同步）
