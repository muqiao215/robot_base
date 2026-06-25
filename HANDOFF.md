# robot_base 交接说明 (HANDOFF)

> 本文档记录 `robot_base` 仓库当前状态、pi 端落地步骤、待验证项与后续工作。
> **机密信息不进仓库**：pi 登录密码 / Tailscale 地址 / 代理 UUID / Reality key / GitHub token
> 一律见本地 `/tmp/raspi-new-handoff.md`，不要提交。

## 1. 仓库当前状态

- 远端：https://github.com/muqiao215/robot_base （public，默认分支 `main`）
- 最新提交：`8a27a1e` — nav2: use fast path-invalid-recovery BT, simplify global costmap, add footprint
- 平台：ROS 2 Jazzy / Python 3.12 / Ubuntu 24.04（Raspberry Pi）
- 仓库结构：`robot_base/`（ROS 2 包）、`map/`（示例栅格地图）、`docs/frames/`（TF 快照）

## 2. 本轮已落地的修复（commit 8a27a1e）

针对“RViz 下目标后 Nav2 反复规划失败、等很久”做的 4 处改动：

| 文件 | 改动 |
| --- | --- |
| `setup.py` | 增加 `data_files` 安装 `config/behavior_trees/*.xml`，否则包内 BT 装不进 share |
| `config/nav2_params.yaml` | `bt_navigator` 的两个 default BT 指向自定义 `nav_to_pose_path_invalid_recovery_fast.xml`（仅路径 invalid 时重规划、恢复 2 次），替换默认 1Hz 周期重规划+恢复树；`global_costmap` 去掉 `obstacle_layer`（仅 static+inflation），加 `robot_radius: 0.30`；`local_costmap` 补 `footprint` 多边形，保留 scan obstacle |
| `launch/nav_all.launch.py` | `params_file` 走 `get_package_share_directory('robot_base')`，`map` 暴露为 `LaunchConfiguration`（去掉源码树绝对路径） |
| `robot_base/drive_node.py` | `allow_cmd_odom_fallback` 改为 `declare_parameter`，MSPD 反馈丢失时可运行时打开 |

BT xml 路径用的是 **install 绝对路径**（`install/robot_base/share/robot_base/config/behavior_trees/...`），因为 ROS 2 yaml 参数**不展开** `$(find-pkg-share)`。文件内已注释。

## 3. ⚠️ 阻塞：pi 端尚未生效

pi（`raspi-new`）在本轮掉线，runtime **还没 rebuild**，改动尚未在实机上生效。
pi 回来后必须执行（`GIT_CONFIG_GLOBAL` 那段见第 6 节说明）：

```bash
# 1) 拉取最新提交
git -C ~/publish/robot_base_repo pull

# 2) 把 4 个改动文件同步到实运行源码树
cp ~/publish/robot_base_repo/robot_base/setup.py                  ~/ros2_ws/src/robot_base/setup.py
cp ~/publish/robot_base_repo/robot_base/config/nav2_params.yaml   ~/ros2_ws/src/robot_base/config/nav2_params.yaml
cp ~/publish/robot_base_repo/robot_base/launch/nav_all.launch.py  ~/ros2_ws/src/robot_base/launch/nav_all.launch.py
cp ~/publish/robot_base_repo/robot_base/robot_base/drive_node.py  ~/ros2_ws/src/robot_base/robot_base/drive_node.py

# 3) 重新构建（setup.py 改了，必须 rebuild 才会装 BT xml）
cd ~/ros2_ws && colcon build --packages-select robot_base

# 4) 验证 BT xml 已进 install
ls ~/ros2_ws/install/robot_base/share/robot_base/config/behavior_trees/
# 期望看到: nav_to_pose_path_invalid_recovery_fast.xml

# 5) 重新启动导航
source ~/ros2_ws/install/setup.bash
ros2 launch robot_base nav_all.launch.py
```

## 4. 待实车验证（代码未改，需人工）

`drive_node.py` 的正运动学有几处疑似经验修正，**本轮没动符号**，请在实车上验证：

```python
vx = -(m1 + m2 + m3 + m4) / 4.0
vy = -(-m1 + m2 + m3 - m4) / 4.0
wz = -(m3 + m4 - m1 - m2) / (self.wheel_base * 2.5)   # 分母 *2.5 不是标准麦轮 FK
```

- 在 RViz/TF 里看 `odom → base_link`：
  - 手动发 `cmd_vel linear.x=0.1`，base_link 是否沿 odom 的 +x 前进；
  - 发 `cmd_vel angular.z=0.3`，角度是否同向。
- 若方向反了，AMCL 与全局规划会越来越乱，表现就是“目标下了但 planner/controller 一直失败”。
- MSPD 反馈经常丢时，临时 `ros2 param set /robot_drive_node allow_cmd_odom_fallback true`，避免每次重启 Nav2。

## 5. 延期工作（本轮明确不做）

- **控制器**：RPP → MPPI 或 DWB，并在 `velocity_smoother` 打开 `vy=±0.12`，让麦克纳姆真正用上横向。等规划不反复失败后再换，不要同时大改 BT/costmap/controller。
- **bt_navigator 超时**：当前保留 `default_server_timeout: 20` / `action_server_result_timeout: 900.0`，未按示例收紧到 5 / 60。
- **全局 costmap obstacle_layer**：基础导航稳定后再回填。

## 6. pi 端操作备忘（非机密）

- `gh` 已登录 `muqiao215`；从 pi 访问 GitHub **必须走代理** `http://127.0.0.1:2080`（sing-box；出口 IP 见本地 `/tmp/raspi-new-handoff.md`，直连被墙）。
- pi 全局 `~/.gitconfig` 有一条 `insteadOf`：`https://github.com/` → `https://gitclone.com/github.com/`（只读镜像）。**push 时必须绕过**，否则 504：
  ```bash
  export https_proxy=http://127.0.0.1:2080
  GIT_CONFIG_GLOBAL=/dev/null git \
    -c credential.https://github.com.helper='!gh auth git-credential' \
    push origin main
  ```
- 本地干净工作副本：`~/publish/robot_base_repo`（已 tracking `origin/main`）。
- **实运行源码树**：`~/ros2_ws/src/robot_base`（只有在这里改 + `colcon build` 才在实机生效）。
- 第三方包（勿改、勿发布）：`~/ros2_ws/src/slam_toolbox`、`~/sllidar_ws/src/sllidar_ros2`。

## 7. 机密信息位置

见本地 `/tmp/raspi-new-handoff.md`（pi 登录、Tailscale、代理节点详情）。
**不要把该文件内容提交到任何仓库。**
