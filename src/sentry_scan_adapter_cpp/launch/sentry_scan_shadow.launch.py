"""RM → SCAN 影子接入入口（I1/I2）：真实输入，**不驱动车辆**。

数据流（全部在 /sentry_scan 命名空间内）：

    RM 话题/TF ──> rm_input_adapter ──> body_pose / sensor_pose / cloud ──> scan_planner_node
                          │                                                     │ planning/bspline
                          │                                                     v
                          │                                        closed_loop_controller
                          │                                                     │ cmd_vel -> cmd_vel_candidate
                          └── health/health_ok ──> shadow_guard <──────────────┘
                                                       │            ▲
                                                       │            └── grid_map/cloud_update（地图真的在更新吗）
                                                       └─> cmd_vel_shadow（只记录/显示，不接车）

    任务侧：RViz/上层目标 ──> task/goal_in ──> task_adapter ──> goal ──> FSM (Mode 1)
            参考路线发布器 ──> task/path_in ──> task_adapter ──> initial_path ──> FSM (Mode 3)

    task_adapter 按消息时间戳把目标/路线从它们的 header.frame_id 转换到规划系；
    FSM 本身不做 frame 变换，因此**不能**把 map 下的坐标直接接进 FSM。

本入口的硬性边界：
  * **没有**“下发真实命令”的开关；不启动 UART、Nav2、LIO、registration、机器人驱动、
    运动模拟器或 PCD 渲染。RM 原有链由它自己的部署入口启动，本文件只订阅其话题与 TF。
  * 不启动 `open_loop_controller`（它直接发布模拟里程计，不是实车接口）。
  * 目标与参考路线一律经过 `task_adapter` 做显式坐标变换；空 frame 或查不到 TF 的输入被拒绝。
    Mode 2 的航点来自参数文件，按约定必须是**规划系**坐标（配置期输入，不做运行期变换）。
  * `shadow_guard` 只创建 `cmd_vel_shadow` 发布者，并周期检查 ROS 图中是否出现
    `/cmd_vel`、`/cmd_vel_remap` 发布者；一旦出现影子输出立即归零。

回放（B3）与在线只读影子的区别：
  * 在线影子：与 RM 共用 ROS domain，只读；
  * 回放：`replay:=true` 时必须显式设置独立的 ROS_DOMAIN_ID，并把录包中的控制话题重映射到
    隔离名称（见 docs/interfaces/shadow_input_contract.md 的回放步骤），禁止把录包命令发到车辆。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = "sentry_scan"


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _params_from_file(yaml_path):
    """读取参数文件里所有节点的 ros__parameters（用于航点/参考路线这类只有一段的文件）。

    这些示例文件里的 key 是 `/sentry_sim/<node>`，直接作为 params-file 传给
    `/sentry_scan/<node>` 不会生效，因此在这里按键名无关的方式取出并作为 dict 覆盖。
    """
    with open(yaml_path) as handle:
        data = yaml.safe_load(handle) or {}
    merged = {}
    for value in data.values():
        if isinstance(value, dict):
            merged.update(value.get("ros__parameters", {}))
    return merged


def _namespace_params(yaml_path, node_name, overrides):
    """读取仿真/共享 YAML 中该节点名的参数块，并叠加影子覆盖。

    复用 `sentry_planner.yaml` / `sentry_controllers.yaml` 作为参数的**唯一来源**，
    只把命名空间从 /sentry_sim 换成 /sentry_scan，避免复制一份会漂移的配置。
    """
    with open(yaml_path) as handle:
        data = yaml.safe_load(handle) or {}
    block = {}
    for key, value in data.items():
        if key.split("/")[-1] == node_name and isinstance(value, dict):
            block.update(value.get("ros__parameters", {}))
    block.update(overrides)
    return block


def _setup(context):
    adapter_share = get_package_share_directory("sentry_scan_adapter_cpp")
    scan_share = get_package_share_directory("scan_planner")
    contract_yaml = os.path.join(adapter_share, "config", "shadow_contract.yaml")
    planner_yaml = os.path.join(scan_share, "config", "sentry_planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "sentry_controllers.yaml")
    rviz_config = os.path.join(adapter_share, "launch", "sentry_scan_shadow.rviz")

    def value(name):
        return LaunchConfiguration(name).perform(context)

    navi_mode = int(value("navi_mode"))
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1 (RViz goal), 2 (waypoints) or 3 (reference path)")

    keypoints_file = value("keypoints_file")
    reference_path_file = value("reference_path_file")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError("navi_mode=2 requires keypoints_file to reference an existing YAML")
    if navi_mode == 3 and (not reference_path_file or not os.path.isfile(reference_path_file)):
        raise RuntimeError("navi_mode=3 requires reference_path_file to reference an existing YAML")
    if navi_mode != 3 and reference_path_file:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")

    planning_frame = value("planning_frame")
    task_frame = value("task_frame")
    body_frame = value("body_frame")
    sensor_frame = value("sensor_frame")
    use_sim_time = _as_bool(value("use_sim_time"))
    replay = _as_bool(value("replay"))
    if replay and not use_sim_time:
        raise RuntimeError("replay:=true requires use_sim_time:=true (bag /clock)")
    if use_sim_time and not replay:
        raise RuntimeError(
            "use_sim_time:=true only allowed with replay:=true (needs a /clock source)")
    if replay:
        domain = os.environ.get("ROS_DOMAIN_ID", "")
        if domain in ("", "0"):
            raise RuntimeError(
                "replay:=true requires an explicit isolated ROS_DOMAIN_ID (not unset/0): the replay "
                "must not share the vehicle's domain")

    robot_height = float(value("robot_height"))
    robot_radius = float(value("robot_radius"))
    safety_margin = float(value("safety_margin"))
    half_h = 0.5 * robot_height
    body_height_arg = value("body_height").strip()
    body_height = float(body_height_arg) if body_height_arg else half_h
    z_up_arg = value("inflation_z_up").strip()
    z_down_arg = value("inflation_z_down").strip()
    z_up = float(z_up_arg) if z_up_arg else half_h
    z_down = float(z_down_arg) if z_down_arg else half_h
    strict_pairing = _as_bool(value("strict_sensor_pairing"))
    pairing_tolerance = float(value("sensor_pairing_tolerance"))
    start_rviz = _as_bool(value("start_rviz"))
    log_dir = os.path.expanduser(value("shadow_log_dir"))

    input_gate = _as_bool(value("input_gate"))
    cmd_gate_enabled = _as_bool(value("cmd_gate"))
    # input_gate:=true 时，适配器改为消费闸门输出；闸门只读实车话题，用于现场验证断流/恢复。
    if input_gate:
        adapter_odom = "test/odom"
        adapter_velocity = "test/velocity"
        adapter_cloud = "test/cloud"
    else:
        adapter_odom = value("odom_topic")
        adapter_velocity = value("velocity_topic")
        adapter_cloud = value("cloud_topic")

    common = {"use_sim_time": use_sim_time}
    nodes = [
        LogInfo(msg="SHADOW MODE (no chassis output): adapter -> SCAN -> candidate velocity; "
                    "shadow_guard only records cmd_vel_shadow. No UART/Nav2/LIO/simulator started."),
        Node(
            package="sentry_scan_adapter_cpp", executable="rm_input_adapter",
            name="rm_input_adapter",
            namespace=NAMESPACE, output="screen",
            parameters=[contract_yaml, common, {
                "planning_frame": planning_frame, "task_frame": task_frame,
                "body_frame": body_frame, "sensor_frame": sensor_frame,
                "velocity_frame": value("velocity_frame"),
                "odom_topic": adapter_odom,
                "velocity_topic": adapter_velocity,
                "cloud_topic": adapter_cloud,
                "max_source_age": float(value("max_source_age")),
                "max_receive_age": float(value("max_receive_age")),
                "tf_future_tolerance": float(value("tf_future_tolerance")),
                "odom_in_planning_frame": _as_bool(value("odom_in_planning_frame")),
                "require_tf": _as_bool(value("require_tf")),
                "require_center_velocity": _as_bool(value("require_center_velocity")),
                "cloud_assume_planning_frame": _as_bool(value("cloud_assume_planning_frame")),
            }],
        ),
        Node(
            package="scan_planner", executable="scan_planner_node", name="scan_planner_node",
            namespace=NAMESPACE, output="screen",
            remappings=[("move_base_simple/goal", "goal")],
            parameters=[
                _namespace_params(planner_yaml, "scan_planner_node", {
                    **common,
                    "fsm.navi_mode": navi_mode,
                    "grid_map.sensor_type": "lidar",
                    "grid_map.frame_id": planning_frame,
                    # adapter 已把云与射线原点转换到规划系，且外参已在 TF 中体现。
                    "grid_map.cloud_is_world": True,
                    "grid_map.need_extrinsic": False,
                    "grid_map.strict_sensor_pairing": strict_pairing,
                    "grid_map.sensor_pairing_tolerance": pairing_tolerance,
                    # 影子阶段不加载地面网格：Mode 1 目标高度沿用初始 body_pose z。
                    "grid_map.ground_grid_file": "",
                    "grid_map.body_height": body_height,
                    "grid_map.double_cylinder_radius": robot_radius,
                    "grid_map.double_cylinder_offset": 0.0,
                    "grid_map.safety_margin": safety_margin,
                    "grid_map.obstacles_inflation_z_up": z_up,
                    "grid_map.obstacles_inflation_z_down": z_down,
                    "visualization_frame_id": planning_frame,
                }),
            ] + ([_params_from_file(keypoints_file)] if keypoints_file else []),
        ),
        Node(
            package="scan_planner", executable="closed_loop_controller", name="closed_loop_controller",
            namespace=NAMESPACE, output="screen",
            parameters=[_namespace_params(controllers_yaml, "closed_loop_controller", {
                **common,
                "yaw_mode": "hold",
                "spin_rate": 0.0,
                # 影子阶段不透传 yaw 候选：MCU 保留朝向所有权。
                "yaw_candidate_enabled": False,
            })],
            remappings=[("cmd_vel", value("controller_cmdvel_topic"))],
        ),
        Node(
            package="sentry_scan_adapter_cpp", executable="shadow_guard", name="shadow_guard",
            namespace=NAMESPACE, output="screen",
            parameters=[contract_yaml, common, {
                "log_dir": log_dir,
                "cloud_update_topic": value("map_update_topic"),
                "max_map_age": float(value("max_map_age")),
            }],
        ),
        Node(
            package="sentry_scan_adapter_cpp", executable="input_pause_gate", name="input_pause_gate",
            namespace=NAMESPACE, output="screen",
            parameters=[contract_yaml, common, {
                "odom_in": value("odom_topic"),
                "velocity_in": value("velocity_topic"),
                "cloud_in": value("cloud_topic"),
            }],
        ) if input_gate else LogInfo(msg="input_gate:=false (断流/恢复验证不可用；"
                                         "用 input_gate:=true 启动可暂停的影子输入)"),
        Node(
            package="sentry_scan_adapter_cpp", executable="cmd_gate", name="cmd_gate",
            namespace=NAMESPACE, output="screen",
            parameters=[common, {
                "candidate_topic": "cmd_vel_candidate",
                "output_topic": value("gate_output_topic"),
                "max_vx": float(value("gate_max_vx")),
                "max_vy": float(value("gate_max_vy")),
                "max_wz": float(value("gate_max_wz")),
                "allow_wz": _as_bool(value("gate_allow_wz")),
            }],
        ) if cmd_gate_enabled else LogInfo(
            msg="cmd_gate:=false (命令闸门未启用；接管测试必须显式启用并设置限幅)"),
        Node(
            package="sentry_scan_adapter_cpp", executable="task_adapter", name="task_adapter",
            namespace=NAMESPACE, output="screen",
            parameters=[contract_yaml, common, {"planning_frame": planning_frame}],
        ),
    ]

    # 影子阶段不加载地面网格，也不预设初始位姿：Mode 1 目标高度沿用初始 body_pose z
    # （grid_map.ground_grid_file 为空时的既有语义），真实初始位姿由输入决定。
    if navi_mode == 3:
        nodes.append(Node(
            package="sentry_scan_adapter_cpp", executable="reference_path_publisher",
            name="reference_path_publisher", namespace=NAMESPACE, output="screen",
            remappings=[("initial_path", "task/path_in")],
            parameters=[_params_from_file(reference_path_file), common],
        ))

    if start_rviz and not os.path.exists(rviz_config):
        # 不静默失败：配置缺失时 rviz2 会打开一个空显示列表，很难排查。
        print("[WARN] start_rviz:=true 但找不到 RViz 配置: %s\n"
              "       请先 `scripts/build.sh` 重新安装（setup.py 需包含 launch/*.rviz）；"
              "或先用 start_rviz:=false 启动。" % rviz_config)
    if start_rviz:
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", namespace=NAMESPACE,
            output="screen", arguments=["-d", rviz_config], parameters=[common],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("navi_mode", default_value="1",
                              description="1=RViz 目标, 2=参数航点, 3=参考路线"),
        DeclareLaunchArgument("keypoints_file", default_value="",
                              description="navi_mode=2 的航点参数 YAML（机体参考中心 XYZ）"),
        DeclareLaunchArgument("reference_path_file", default_value="",
                              description="navi_mode=3 的参考路线参数 YAML（地面 z）"),
        DeclareLaunchArgument("planning_frame", default_value="odom"),
        DeclareLaunchArgument("task_frame", default_value="map"),
        DeclareLaunchArgument("body_frame", default_value="base_link"),
        DeclareLaunchArgument("sensor_frame", default_value="lidar_link"),
        DeclareLaunchArgument("velocity_frame", default_value="world"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry_transformed"),
        DeclareLaunchArgument("velocity_topic", default_value="/LIVO2/imu_propagate"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("max_source_age", default_value="0.5"),
        DeclareLaunchArgument("max_receive_age", default_value="0.5"),
        DeclareLaunchArgument("tf_future_tolerance", default_value="0.15",
                              description="同一时刻 TF 与消息并发到达时，允许用最新 TF 顶替的最大偏差"),
        DeclareLaunchArgument("odom_in_planning_frame", default_value="true",
                              description="里程计消息已在规划系时直接用其位姿"
                                          "（避免依赖先发消息后发 TF 的动态查询）"),
        DeclareLaunchArgument("require_tf", default_value="true"),
        DeclareLaunchArgument("require_center_velocity", default_value="false",
                              description="true = 没有杆臂补偿时拒绝速度（twist 置零）"),
        DeclareLaunchArgument("cloud_assume_planning_frame", default_value="false"),
        DeclareLaunchArgument("robot_height", default_value="0.25"),
        DeclareLaunchArgument("robot_radius", default_value="0.26"),
        DeclareLaunchArgument("safety_margin", default_value="0.0",
                              description="碰撞包络余量；用户未给出，保持 0，不当作实车阈值"),
        DeclareLaunchArgument("body_height", default_value="",
                              description="机体参考中心相对地面高度；留空 = robot_height/2"),
        DeclareLaunchArgument("inflation_z_up", default_value="", description="留空 = robot_height/2"),
        DeclareLaunchArgument("inflation_z_down", default_value="", description="留空 = robot_height/2"),
        DeclareLaunchArgument("strict_sensor_pairing", default_value="true",
                              description="GridMap 要求云与其射线原点按时间戳配对"),
        DeclareLaunchArgument("sensor_pairing_tolerance", default_value="0.02"),
        DeclareLaunchArgument("map_update_topic", default_value="grid_map/cloud_update",
                              description="规划地图接受新点云的心跳话题"),
        DeclareLaunchArgument("max_map_age", default_value="0.5",
                              description="心跳超过该时间影子输出归零"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("cmd_gate", default_value="false",
                              description="true = 启动命令闸门（默认禁止输出，需显式使能）"),
        DeclareLaunchArgument("controller_cmdvel_topic", default_value="cmd_vel_candidate",
                              description="跟踪器速度输出去向：默认 cmd_vel_candidate（经命令闸门）；"
                                          "设为 /cmd_vel 则**绕过闸门直连底盘**，"
                                          "仅在轮子离地的台架测试中使用，落地必须改回默认"),
        DeclareLaunchArgument("gate_output_topic", default_value="cmd_vel_gated",
                              description="闸门输出话题；默认不是 /cmd_vel，误启动不会驱动底盘"),
        DeclareLaunchArgument("gate_max_vx", default_value="0.0"),
        DeclareLaunchArgument("gate_max_vy", default_value="0.0"),
        DeclareLaunchArgument("gate_max_wz", default_value="0.0"),
        DeclareLaunchArgument("gate_allow_wz", default_value="false"),
        DeclareLaunchArgument("input_gate", default_value="false",
                              description="true = 在实车话题与适配器之间插入可暂停的输入闸门"
                                          "（现场断流/恢复验证；只影响 /sentry_scan）"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="仅回放时为 true，且必须有 /clock"),
        DeclareLaunchArgument("replay", default_value="false",
                              description="true = 回放模式：要求隔离的 ROS_DOMAIN_ID 与 use_sim_time"),
        DeclareLaunchArgument("shadow_log_dir", default_value="log/shadow",
                              description="影子候选速度记录目录（相对当前工作目录）"),
        OpaqueFunction(function=_setup),
    ])
