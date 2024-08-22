from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)
from launch.conditions import IfCondition
from os import path
from ament_index_python.packages import get_package_share_directory


def generate_demo_launch(moveit_config, launch_package_path=None):

    # 如果没有指定 launch_package_path，就使用 moveit_config.package_path，即 `moveit_config` 的包路径
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    world_param = DeclareLaunchArgument(
        "world",
        default_value=path.join(
            get_package_share_directory("aubo_moveit_config"),
            "worlds",
            "follow_target.sdf",
        ),
        description="Name or filepath of world to load.",
    )
    ld.add_action(world_param)

    world = LaunchConfiguration("world")

    # TODO: 1. xacro转sdf
    # xacro2sdf = ExecuteProcess(
    #     cmd=[
    #         PathJoinSubstitution([FindExecutable(name="ros2")]),
    #         "run",
    #         "aubo_description",
    #         "xacro2sdf.bash",
    #         ["ros2_control:=", "true"],
    #         ["ros2_control_plugin:=", "gz"],
    #         ["ros2_control_command_interface:=", "position"],
    #     ],
    #     shell=True,
    # )
    # ld.add_action(xacro2sdf)

    # 声明一个布尔类型的 launch 参数，用于控制是否启用调试模式
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    # 声明一个布尔类型的 launch 参数，用于控制是否启用 RViz
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    # 如果有虚拟关节，则通过包含 virtual_joints launch 来广播 static tf
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # TODO: 2. 启动 move_group 节点
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    # TODO: 3.运行 Rviz 并加载默认配置以查看 move_group 节点的状态
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 启动控制器
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # Given the published joint states, publish tf for the robot links and the robot description
    # TODO: 4.给定已发布的关节状态，发布机器人坐标系的 tf 和机器人描述
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 50.0, "use_sim_time": True},
        ],
    )
    ld.add_action(rsp_node)

    # TODO: 5.启动gazebo仿真
    ld.add_action(
        IncludeLaunchDescription(  # 在一个 launch 文件中包含另一个 launch 文件
            PythonLaunchDescriptionSource(  # 指定被包含的 launch 文件的路径
                PathJoinSubstitution(  # 将多个路径片段连接在一起，生成一个完整的路径，相比os.path.join()可以结合其他 ROS2 的 substitution 类（如 FindPackageShare）动态生成路径，适用于 ROS 2 的 launch 文件。
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[("gz_args", [world, " -r -v ", "3"])],
        )
    )

    # TODO: 6.gazebo中创建机器人模型和ros2_bridge
    create_model = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("aubo_description"),
                    "urdf/model.sdf",
                ]
            ),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[{"use_sim_time": True}],
    )
    ld.add_action(create_model)

    # create_board = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="log",
    #     arguments=[
    #         "-file",
    #         "/home/xvshuo/work/scl/gazebo_moviet2/src/Calibration Plane/calibration_plane.sdf",
    #         "--ros-args",
    #         "--log-level",
    #         "warn",
    #     ],
    #     parameters=[{"use_sim_time": True}],
    # )
    # ld.add_action(create_board)

    clock_bridge = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("aubo_description"),
                    "urdf/model.sdf",
                ]
            ),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[{"use_sim_time": True}],
    )
    ld.add_action(clock_bridge)

    target_pose_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            "/model/target/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[("/model/target/pose", "/target_pose")],
    )
    ld.add_action(target_pose_bridge)

    camrea_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="log",
    )
    ld.add_action(camrea_bridge)

    # ros_gz_sim_create = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=xacro2sdf,
    #         on_exit=[
    #             Node(
    #                 package="ros_gz_sim",
    #                 executable="create",
    #                 output="log",
    #                 arguments=[
    #                     "-file",
    #                     PathJoinSubstitution(
    #                         [
    #                             FindPackageShare("aubo_description"),
    #                             "urdf/model.sdf",
    #                         ]
    #                     ),
    #                     "--ros-args",
    #                     "--log-level",
    #                     "warn",
    #                 ],
    #                 parameters=[{"use_sim_time": True}],
    #             ),
    #             # ros_ign_bridge (clock -> ROS 2)
    #             Node(
    #                 package="ros_ign_bridge",
    #                 executable="parameter_bridge",
    #                 output="log",
    #                 arguments=[
    #                     "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    #                     "--ros-args",
    #                     "--log-level",
    #                     "warn",
    #                 ],
    #                 parameters=[{"use_sim_time": True}],
    #             ),
    #             # ros_ign_bridge (target pose -> ROS 2)
    #             # /model/target/pose" 是一个ROS话题名称
    #             # @ 是一个分隔符，用于分隔ROS话题名称和消息类型
    #             # "geometry_msgs/msg/PoseStamped[ignition.msgs.Pose" 指定了ROS消息类型和Ignition消息类型之间的映射，表示
    #             # 连在一起的两个消息类型之间的映射关系：ROS消息类型是 geometry_msgs/PoseStamped，话题是 /model/target/pose，Ignition消息类型是 ignition.msgs.Pose，方向是从Ignition到ROS。
    #             Node(
    #                 package="ros_ign_bridge",
    #                 executable="parameter_bridge",
    #                 output="log",
    #                 arguments=[
    #                     "/model/target/pose"
    #                     + "@"
    #                     + "geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
    #                     "--ros-args",
    #                     "--log-level",
    #                     "warn",
    #                 ],
    #                 parameters=[{"use_sim_time": True}],
    #                 remappings=[("/model/target/pose", "/target_pose")],
    #             ),
    #             # Camera Bridge
    #             Node(
    #                 package="ros_gz_bridge",
    #                 executable="parameter_bridge",
    #                 arguments=[
    #                     "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
    #                     "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #                 ],
    #                 output="log",
    #             ),
    #         ],
    #     )
    # )
    # ld.add_action(ros_gz_sim_create)

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "aubo_i10", package_name="aubo_moveit_config"
    ).to_moveit_configs()
    ld = generate_demo_launch(moveit_config)

    return ld
