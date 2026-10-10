# scan_VI_26_Sentry —— SCAN-Planner 实车适配精简版

这是从迁移工作区 `adapt_VI_26_Sentry` 抽出的**可部署最小集合**：只保留 SCAN-Planner 核心、
RM→SCAN 实车适配层、RViz 配置与启动入口。**不含**仿真包（Gazebo/mockamap/开环预览）、
文档、实验脚本与历史结果。

- 运行链实现语言：**全 C++**（SCAN 本体 + 6 个适配节点）
- 构建系统：ROS 2 Humble + colcon（ament_cmake）
- 默认**不向底盘输出任何速度**：影子模式下速度只出现在 `/sentry_scan/cmd_vel_shadow`

---

## 1. 架构与数据流

```
  实车 (RM)                              SCAN-Planner (本仓库)                      输出
  ─────────                              ────────────────────                       ────
  /Odometry_transformed ─┐
  /LIVO2/imu_propagate  ─┼─> rm_input_adapter ─> body_pose / sensor_pose / cloud ─┐
  /cloud_registered     ─┘        │                                             │
                                  └─> health_ok / health / planning/reset        │
  TF: odom→base_link, odom→camera_init（静态单位）                                │
                                                                                 v
  RViz 2D Goal Pose ─> task/goal_in ─> task_adapter ─> goal ──┐            scan_planner_node
  参考路线     ─> task/path_in ─> task_adapter ─> initial_path ┤            （FSM 三模式）
                                                              ├──> grid_map ──> A*/B样条优化
                                                              │
                                                              v
                                                    closed_loop_controller
                                                    （全向跟踪器）
                                                              │ cmd_vel
                                                              v
                                                              │
                                                              v
                                        /cmd_vel（跟踪器直连）──> uart_node ──> 电控
```

**本副本是"直连实车"版本**：`closed_loop_controller` 直接发布 `/cmd_vel` → `uart_node` → 电控。
已按用户要求**移除**影子门控（`shadow_guard`）、命令闸门（`cmd_gate`）与断流闸门（`input_pause_gate`）
及其限幅/默认拒发/锁止逻辑。**没有任何软件层阻拦速度输出**，接管前请自行确认单一速度源与急停。

保留的是 SCAN 原生机制（不属于我们加的安全层）：任务授权 `planning/task_active`、
输入健康 `health_ok` 与 `planning/reset`（输入失效时撤销任务），它们不影响正常开车。

---

## 2. 目录与文件作用

```
src/planner/                    ← SCAN-Planner 核心（上游 SCAN-Planner-Ros2 的实现副本）
├── scan_planner_msgs/          自定义消息
│   └── msg/{Bspline,DataDisp,TaskAuthorization}.msg
│        Bspline：跟踪器接收的轨迹；DataDisp：可视化数据；TaskAuthorization：任务授权(task_id)
├── plan_env/                   栅格地图与射线
│   ├── src/grid_map.cpp        占据栅格：点云融合、无效云拒绝、严格传感器配对、
│   │                           膨胀包络（圆柱半径/高度）、发布 grid_map/{occupancy,occupancy_inflate,
│   │                           unknown,cloud_update}；cloud_update 是"地图真的在更新"的心跳
│   └── src/raycast.cpp         射线投射（自由空间/障碍标记）
├── path_searching/
│   └── src/dyn_a_star.cpp      动力学 A*（前端搜索，考虑加速度/速度上限）
├── bspline_opt/                后端轨迹优化
│   ├── src/uniform_bspline.cpp        B 样条表示（位置/速度/加速度求值）
│   ├── src/bspline_optimizer.cpp      代价函数与梯度优化（平滑/避障/可行性）
│   └── src/gradient_descent_optimizer.cpp  优化器实现
├── traj_utils/
│   ├── src/planning_visualization.cpp 可视化发布（optimal_list/goal_point/global_list 等 Marker）
│   └── src/polynomial_traj.cpp        多项式轨迹工具
└── plan_manage/                状态机与执行
    ├── src/scan_replan_fsm.cpp        三模式状态机：
    │                                  Mode 1 = 单目标(rviz goal)、Mode 2 = 参数航点、Mode 3 = 参考路线；
    │                                  负责重规划、取消/复位、发布 TaskAuthorization
    ├── src/planner_manager.cpp        规划管理器（A* → B 样条 → 可行性检查 → 输出轨迹）
    ├── src/scan_planner_node.cpp      节点入口（参数、话题接线）
    ├── src/closed_loop_controller.cpp 全向闭环跟踪器：接收 Bspline → 发布 cmd_vel（默认话题名 cmd_vel）
    ├── src/go2_gait_publisher.cpp     四足步态发布（本实车不使用）
    ├── src/go2_kinematic_sim.cpp      四足运动学仿真（**不用于实车**）
    ├── src/open_loop_controller.cpp   开环里程计模拟（**不用于实车**，会发布假 odom）
    ├── launch/                        上游 launch（仿真用；实车请用下节的适配层 launch）
    ├── config/                        SCAN 参数（栅格分辨率、膨胀、速度/加速度上限、规划权重…）
    └── scripts/reference_path_publisher.py  上游 Python 参考路线发布器（本仓库改用 C++ 版本）

src/sentry_scan_adapter_cpp/    ← 实车适配层（本仓库新增，全 C++）
├── src/rm_input_adapter_node.cpp        实车输入→SCAN：
│          （真实 TfTransformer 先发 odom、后广播同 stamp 的 TF，按消息时刻查会被误拒）
│          （xyz 字段/FLOAT32/count/offset/大端/row_step/data 长度），再做刚体变换
├── src/task_adapter_node.cpp            目标与路线坐标转换：任意 frame → 规划系（未知/空 frame 拒绝）
├── src/reference_path_publisher_node.cpp Mode 3 参考路线发布（参数 waypoints，等 body_pose 后周期发布）
├── （本副本已移除）shadow_guard / cmd_gate / input_pause_gate 三个安全节点
├── launch/sentry_scan_shadow.launch.py  实车启动入口（见第 3 节）
├── launch/sentry_scan_shadow.rviz       RViz 配置（真实云/适配云/体素占据/膨胀/包络/轨迹/目标）
├── config/shadow_contract.yaml          适配层与门控的参数模板
├── config/shadow_test_waypoints.yaml    Mode 2 测试航点
├── config/shadow_test_reference_path.yaml Mode 3 测试路线
└── test/test_cloud_layout_logic.cpp     点云布局反例单测（缺字段/错类型/截断/大端等）
```

---

## 3. 构建与启动

### 构建
```bash
cd ~/scan_VI_26_Sentry
./build.sh                       # colcon build --symlink-install（Release）
source install/setup.bash        # 每个新终端都要 source
```

### 启动（直连实车）
```bash
# 先按原有方式启动实车：LIO(laserMapping)、点云、TfTransformer、uart_node
# 然后：
ros2 launch sentry_scan_adapter_cpp sentry_scan_shadow.launch.py start_rviz:=true
# 启动后：跟踪器直接向 /cmd_vel 输出速度 → uart_node → 电控
```
常用参数（`ros2 launch sentry_scan_adapter_cpp sentry_scan_shadow.launch.py --show-args` 看全部）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `start_rviz` | false | 是否同时启动 RViz（现场建议 true） |
| `planning_frame` | odom | 规划系（现场已确认用 odom） |
| `odom_topic` / `velocity_topic` / `cloud_topic` | 实车三话题 | 输入 |
| `robot_height` / `robot_radius` | 0.15 / 0.26 | 碰撞包络（现场实测确认的圆柱尺寸） |
| `safety_margin` | 0.0 | 额外余量 |
| `navi_mode` | 1 | 1=目标、2=参数航点、3=参考路线 |
| `controller_cmdvel_topic` | `/cmd_vel` | 跟踪器输出去向（本副本默认直连底盘） |
| `shadow_guard` / `cmd_gate` / `input_gate` | 无 | 本副本已移除这些安全节点 |

### 台架/落地前的检查（本副本无软件限幅，务必先做）
```bash
# 单一速度源：应只有 /sentry_scan/closed_loop_controller 一个发布者
ros2 topic info -v /cmd_vel
# 停车手段：Ctrl-C 影子入口即停止发布；电控侧是否自动归零取决于 MCU 看门狗（见第 6 节实测）
```

---

## 4. RViz 说明

固定坐标系 = `odom`。已配置的关键显示：
- **RM cloud (/cloud_registered, real)**：真实点云（橙色）
- **Sensor Cloud**：适配后进入 SCAN 的云（白色）
- **Occupancy**：体素占据（青色方块，0.05 m）
- **Inflated Occupancy**：膨胀包络（橙色半透明，0.06 m）
- **SCAN path (optimal_list Marker)**：规划出的轨迹
- **Sentry envelope + heading**：碰撞包络与机头方向
- **RM odom**：真实机体位姿（箭头）
- 2D Goal Pose 已指向 `/sentry_scan/task/goal_in`（**不会**触发 Nav2）

---

## 5. 实车已确认的事实（现场实测）

| 项 | 值 |
|---|---|
| 输入话题 | `/Odometry_transformed`(odom, ~10 Hz)、`/LIVO2/imu_propagate`(world, ~185 Hz)、`/cloud_registered`(camera_init, ~10 Hz, point_step=48) |
| TF | 动态 `odom→base_link`、`map→odom`；静态 `odom→camera_init`(单位)、`base_link→base_footprint`、`base_footprint→lidar_link` |
| 规划系 | `odom`（已确认） |
| 碰撞包络 | 半径 0.26 m、高 0.15 m 的圆柱（已确认，**不得为通过而缩小**） |
| 速度语义 | `imu_propagate` 的 `world` **不在 TF 中** → 速度前馈按设计降级为零；候选速度由位置闭环产生 |
| 底盘入口 | `/cmd_vel`（`uart_node` 与 `TfTransformer` 订阅，`geometry_msgs/Twist`，RELIABLE/VOLATILE） |
| 旋转语义 | RM `uart_node` 把 `linear.z` 当 yaw（仅特殊区域/SCAN 接管分支），`angular.z` 不被使用 |
| 电控接管前置 | `uart_node` 的 `control_id=25` 只在 `is_in_special_area` 或 `enable_scan_control_` 成立时下发；否则电控可能忽略速度——**开车前必须确认该条件成立** |

---

## 6. 安全与停车条件

- 影子模式**不会**向底盘输出；启动日志里会明确写 `SHADOW MODE (no chassis output)`。
- 本副本**没有**软件限幅/默认拒发/超时归零；安全完全依赖：① 单一速度源检查；
  ② 安全员守急停；③ 现场实测过的 MCU 看门狗。
- 接管前必须现场实测（不可引用仿真值）：
  1. 急停方式的停车时间；
  2. **MCU 在速度停发后多久归零**；
  3. 影子入口进程被杀/崩溃时的停车表现；
  4. 取消（`planning/reset`）后的停车时限；
  5. `health_ok=false` 或地图停更后的停车时限。
- 首轮只做**平坦空旷区域**低速（建议 ≤0.2 m/s），不测洞口/坡道/小陀螺。

---

## 7. 已知限制

- `world` 速度语义未解析 → 速度前馈为 0（不影响位置闭环规划，但接管后的速度环质量未验证）。
- 旋转当前不透传（跟踪器 `yaw_candidate_enabled=false`；RM 的 yaw 走 `linear.z`）。
- 规划出路径 ≠ 车辆能安全通过；包络/几何参考点已目视确认，但未做标定级验证。
- 本副本不含仿真包与历史文档；完整证据与决策记录在迁移工作区 `~/adapt_VI_26_Sentry`。
