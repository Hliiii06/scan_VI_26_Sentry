"""全向哨兵 SCAN 闭环仿真入口（PCD 地图 + RViz）。

默认 closed_loop 数据流（速度闭环）：

    PCD 地图 ──> 局部雷达渲染 ──> SCAN 地图/规划 ──> 全向跟踪速度
        ▲                                                    │
        └────────── 速度积分得到模拟 odom（body_pose）────────┘

所有节点都在 /sentry_sim 命名空间下，cmd_vel 只出现在该命名空间内，
不会与 RM 的 /cmd_vel 或实车 UART 产生任何连接。

可选 waypoint_z_preview 仅用于 Mode 2 无支撑面诊断：直接将样条位姿
写入模拟 odom，不启动速度跟踪/积分器，不验证轮地接触或动力学。

三种输入模式与上游一致，通过 navi_mode 选择：
    1  RViz "2D Goal Pose" 指定目标
    2  keypoints_file 参数航点
    3  reference_path_file 参考路线 + reference_path_publisher
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = "sentry_sim"


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "sentry_planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "sentry_controllers.yaml")
    simulator_yaml = os.path.join(scan_share, "config", "sentry_simulator.yaml")
    rviz_config = os.path.join(scan_share, "rviz", "sentry_sim.rviz")

    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1 (RViz goal), 2 (waypoints) or 3 (reference path)")

    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError("navi_mode=2 requires keypoints_file to reference an existing YAML")
    if navi_mode == 3 and (not reference_path_file or not os.path.isfile(reference_path_file)):
        raise RuntimeError(
            "navi_mode=3 requires reference_path_file to reference an existing YAML")
    if navi_mode != 3 and reference_path_file:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")

    # 初始位姿先读出来，Mode 2 的退化首航点检查需要它。
    init_x = float(LaunchConfiguration("init_x").perform(context))
    init_y = float(LaunchConfiguration("init_y").perform(context))
    init_z = float(LaunchConfiguration("init_z").perform(context))
    init_yaw = float(LaunchConfiguration("init_yaw").perform(context))

    # Mode 2 的第一个航点若与初始位姿重合，多项式全局轨迹退化、规划必然失败
    # （现象是反复重规划后急停）。这里提前报错，而不是让它静默失败。
    if navi_mode == 2:
        import yaml
        with open(keypoints_file) as handle:
            params = yaml.safe_load(handle) or {}
        waypoints = None
        for node_params in params.values():
            if isinstance(node_params, dict) and "ros__parameters" in node_params:
                candidate = node_params["ros__parameters"].get("fsm.waypoints")
                if candidate is not None:
                    waypoints = candidate
                    break
        if not waypoints or len(waypoints) % 3 != 0:
            raise RuntimeError(
                "%s 必须包含 fsm.waypoints，且元素个数为 3 的倍数" % keypoints_file)
        dx = waypoints[0] - init_x
        dy = waypoints[1] - init_y
        dz = waypoints[2] - init_z
        if (dx * dx + dy * dy + dz * dz) < 0.2 ** 2:
            raise RuntimeError(
                "Mode 2 的第一个航点 (%.2f, %.2f, %.2f) 与初始位姿 (%.2f, %.2f, %.2f) 重合，"
                "全局轨迹会退化。请删掉与起点重合的首航点。"
                % (waypoints[0], waypoints[1], waypoints[2], init_x, init_y, init_z))

    pcd_map_file = os.path.expanduser(LaunchConfiguration("pcd_map_file").perform(context))
    if not pcd_map_file or not os.path.isfile(pcd_map_file):
        raise RuntimeError(
            "pcd_map_file must reference an existing PCD file; got '%s'. "
            "本仿真不提供演示地图替换，缺少用户 PCD 时请显式指定 use_mockamap 的开发分支。"
            % pcd_map_file)

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    if use_sim_time:
        raise RuntimeError(
            "use_sim_time=true 需要 /clock 来源；本仿真使用系统时钟，请保持 false")

    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))
    ground_grid_file = os.path.expanduser(LaunchConfiguration("ground_grid_file").perform(context))
    ground_file = os.path.expanduser(LaunchConfiguration("ground_file").perform(context))
    execution_mode = LaunchConfiguration("execution_mode").perform(context)
    if execution_mode not in ("closed_loop", "waypoint_z_preview"):
        raise RuntimeError("execution_mode must be closed_loop or waypoint_z_preview")
    if execution_mode == "waypoint_z_preview" and (ground_grid_file or ground_file):
        raise RuntimeError("waypoint_z_preview forbids ground_grid_file/ground_file: no support surface")
    robot_radius = float(LaunchConfiguration("robot_radius").perform(context))
    # robot_height 是唯一的高度旋钮：机体高 H -> 机体中心离地 H/2，
    # 碰撞包络上下各 H/2（查询点在机体中心）。inflation_z_* 留空时自动取 H/2，
    # 这样"降高度验证可通行性"只需改一个参数，不会出现高度与包络不一致。
    robot_height = float(LaunchConfiguration("robot_height").perform(context))
    half_h = 0.5 * robot_height
    z_up_arg = LaunchConfiguration("inflation_z_up").perform(context)
    z_down_arg = LaunchConfiguration("inflation_z_down").perform(context)
    z_up = float(z_up_arg) if z_up_arg.strip() else half_h
    z_down = float(z_down_arg) if z_down_arg.strip() else half_h
    if ground_grid_file and not os.path.isfile(ground_grid_file):
        raise RuntimeError("ground_grid_file 不存在: %s" % ground_grid_file)
    if ground_file and not os.path.isfile(ground_file):
        raise RuntimeError("ground_file 不存在: %s" % ground_file)
    yaw_mode = LaunchConfiguration("yaw_mode").perform(context)
    if yaw_mode not in ("hold", "align", "spin"):
        raise RuntimeError("yaw_mode must be 'hold', 'align' or 'spin'")
    spin_rate = float(LaunchConfiguration("spin_rate").perform(context))

    common = {"use_sim_time": use_sim_time}
    nodes = []

    # 地图预处理参数。默认值对应"演示地图"：把 rmuc2026_field.pcd 整体下移 0.30 m
    # 并按绝对高度删掉低处点。这是按高度切图，不是地面识别，会连带删除矮障碍，
    # 因此默认同时发布未过滤的对照云，且不可直接带到实车。
    map_params = {
        "file_name": pcd_map_file,
        "map_offset_z": float(LaunchConfiguration("map_offset_z").perform(context)),
        "keep_z_min": float(LaunchConfiguration("keep_z_min").perform(context)),
        "keep_z_max": float(LaunchConfiguration("keep_z_max").perform(context)),
        "publish_raw_cloud": _as_bool(LaunchConfiguration("publish_raw_cloud").perform(context)),
        "ground_file": ground_file,
        # 地形实体网格（按高度着色的三角面），用于在 RViz 里看清坡道与洞口
        "ground_mesh_grid_file": ground_grid_file,
    }

    # 1) 全局 PCD 地图发布（含演示用高度过滤与对照云）
    nodes.append(
        Node(
            package="map_generator",
            executable="map_pub",
            name="map_pub",
            namespace=NAMESPACE,
            output="screen",
            parameters=[simulator_yaml, common, map_params],
        )
    )

    # 2) 局部雷达渲染：按 body_pose 与真实雷达外参生成 world 系点云
    nodes.append(
        Node(
            package="local_sensing_node",
            executable="pcl_render_node",
            name="pcl_render_node",
            namespace=NAMESPACE,
            output="screen",
            parameters=[simulator_yaml, common, {
                "preserve_map_geometry": _as_bool(LaunchConfiguration("preserve_map_geometry").perform(context)),
            }],
            remappings=[("global_map", "/%s/global_cloud" % NAMESPACE)],
        )
    )

    # 3) SCAN 规划节点（FSM + GridMap + 优化）
    planner_overrides = {
        **common,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": "lidar",
        # 渲染出的点云已是 world 系，且外参已在渲染端应用，这里不能再叠一次。
        "grid_map.cloud_is_world": True,
        "grid_map.need_extrinsic": False,
        "grid_map.ground_grid_file": ground_grid_file,
        "grid_map.body_height": half_h,
        # z 向膨胀：查询点在机体中心，机器人高 0.25 m -> 上下各 0.125 m 才与机体等高。
        # 做成 launch 参数是为了能复现地做对照实验（改小会让碰撞检查**低估**机体，
        # 属于放宽安全边界，不是修 bug）。
        "grid_map.obstacles_inflation_z_up": z_up,
        "grid_map.obstacles_inflation_z_down": z_down,
    }
    nodes.append(
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            namespace=NAMESPACE,
            output="screen",
            parameters=[planner_yaml]
            + ([keypoints_file] if keypoints_file else [])
            + [planner_overrides],
            # 渲染节点用 <quadrotor_name>/lidar_pose 绝对话题发布传感器位姿。
            remappings=[("sensor_pose", "/%s/lidar_pose" % NAMESPACE)],
        )
    )

    # 4) 全向跟踪器：输出机体系 vx/vy 与 yaw rate
    execution_start = len(nodes)
    nodes.append(
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            namespace=NAMESPACE,
            output="screen",
            parameters=[controllers_yaml, common,
                        {"yaw_mode": yaw_mode, "spin_rate": spin_rate}],
        )
    )

    # 5) 运动模拟器：积分速度得到 body_pose，构成闭环
    nodes.append(
        Node(
            package="scan_planner",
            executable="go2_kinematic_sim",
            name="go2_kinematic_sim",
            namespace=NAMESPACE,
            output="screen",
            parameters=[controllers_yaml, common,
                        {"init_x": init_x, "init_y": init_y, "init_z": init_z,
                         "init_yaw": init_yaw, "publish_tf": False,
                         "ground_grid_file": ground_grid_file,
                         # 实体机体标记与机体中心高度都跟 robot_height 一致
                         "robot_radius": robot_radius, "body_height": half_h}],
        )
    )

    if execution_mode == "waypoint_z_preview":
        if navi_mode != 2 or yaw_mode != "hold":
            raise RuntimeError("waypoint_z_preview requires Mode 2 and yaw_mode=hold")
        # Replace BOTH velocity tracker and integrator: exactly one odom source.
        del nodes[execution_start:]
        nodes.append(LogInfo(msg="WAYPOINT-Z PREVIEW: no ground grid, direct spline odometry; "
                                 "NOT wheel-contact/velocity-tracking validation. Stop with Ctrl-C."))
        nodes.append(Node(
            package="scan_planner", executable="open_loop_controller",
            name="open_loop_controller", namespace=NAMESPACE, output="screen",
            parameters=[common, {"frame_id": "world", "child_frame_id": "base",
                "init_x": init_x, "init_y": init_y, "init_z": init_z,
                "init_yaw": init_yaw, "publish_rate": 100.0,
                # Existing controller holds yaw below this speed threshold.
                "yaw_min_speed": 1.0e6, "hold_final_position": True}],
        ))

    # 6) Mode 3 的参考路线发布器
    if navi_mode == 3:
        nodes.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                namespace=NAMESPACE,
                output="screen",
                parameters=[reference_path_file, common],
            )
        )

    if start_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=NAMESPACE,
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[common],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "navi_mode", default_value="1",
                description="1=RViz 目标, 2=参数航点, 3=参考路线"),
            DeclareLaunchArgument("execution_mode", default_value="closed_loop",
                description="closed_loop=速度闭环；waypoint_z_preview=无地面网格的三维样条位姿预览，非物理仿真"),
            DeclareLaunchArgument(
                "pcd_map_file", default_value="~/pcd_map/rmuc2026_field.pcd",
                description="用户提供的 PCD 地图路径"),
            DeclareLaunchArgument(
                "map_offset_z", default_value="-0.30",
                description="演示地图：把地图整体下移，使可行驶地面落在 z=0。"
                            "合成测试地图应设为 0.0"),
            DeclareLaunchArgument(
                "keep_z_min", default_value="0.0",
                description="演示地图：删除该高度以下的点（按绝对高度切图，不是地面识别，"
                            "会连带删除矮障碍）。合成测试地图按需设置"),
            DeclareLaunchArgument(
                "keep_z_max", default_value="1.5",
                description="演示地图：删除该高度以上的点"),
            DeclareLaunchArgument(
                "ground_grid_file", default_value="",
                description="地面高度网格（prepare_terrain_map.py / make_terrain_maps.py 产出）。"
                            "给出后运动模拟器与 Mode 1 目标高度都跟随地形；"
                            "留空时 closed_loop 固定 z，waypoint_z_preview 按样条 z"),
            DeclareLaunchArgument(
                "ground_file", default_value="",
                description="地形表面点云，仅用于 RViz 显示地形起伏"),
            DeclareLaunchArgument(
                "robot_height", default_value="0.25",
                description="机器人总高（米）。决定机体中心高度与碰撞包络："
                            "上下各 0.5*robot_height。降高度做可通行性排查时只改这一个"),
            DeclareLaunchArgument(
                "inflation_z_up", default_value="",
                description="z 向**向上**膨胀（米）。留空 = robot_height/2"),
            DeclareLaunchArgument(
                "inflation_z_down", default_value="",
                description="z 向**向下**膨胀（米）。留空 = robot_height/2"),
            DeclareLaunchArgument(
                "robot_radius", default_value="0.26",
                description="机器人外接半径（米），用于实体机体标记；与碰撞包络同源"),
            DeclareLaunchArgument(
                "publish_raw_cloud", default_value="true",
                description="额外发布未过滤的对照云 /sentry_sim/global_cloud_raw"),
            DeclareLaunchArgument("keypoints_file", default_value="",
                                  description="navi_mode=2 的航点参数 YAML"),
            DeclareLaunchArgument("preserve_map_geometry", default_value="true",
                                  description="局部观测保留源点位置，避免极坐标补点凭空加厚洞顶；非真实雷达噪声模型"),
            DeclareLaunchArgument("reference_path_file", default_value="",
                                  description="navi_mode=3 的参考路线参数 YAML"),
            DeclareLaunchArgument("yaw_mode", default_value="hold",
                                  description="hold | align | spin"),
            DeclareLaunchArgument("spin_rate", default_value="0.0",
                                  description="yaw_mode=spin 时的自转角速度 (rad/s)"),
            DeclareLaunchArgument("init_x", default_value="-6.0"),
            DeclareLaunchArgument("init_y", default_value="1.25"),
            DeclareLaunchArgument("init_z", default_value="0.125"),
            DeclareLaunchArgument("init_yaw", default_value="0.0"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false",
                description="本仿真无 /clock 来源，必须保持 false"),
            OpaqueFunction(function=_setup),
        ]
    )
