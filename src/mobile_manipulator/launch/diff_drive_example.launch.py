import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    ignition_ros2_control_demos_path = os.path.join(
        get_package_share_directory("mobile_manipulator")
    )

    xacro_file = os.path.join(
        ignition_ros2_control_demos_path, "urdf", "simple_diff_drive.xacro.urdf"
    )

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml(), "use_sim_time": use_sim_time}

    print(params)

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    robot_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
        output="screen",
    )

    # Bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory("mobile_manipulator"),
                    "config",
                    "ros_gz_bridge.yaml",
                ),
            }
        ],
    )

    world = os.path.join(
        get_package_share_directory("moveit_assistant"), "worlds", "empty.sdf"
    )

    return LaunchDescription(
        [
            ros_gz_bridge,
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(  # 将多个路径片段连接在一起，生成一个完整的路径，相比os.path.join()可以结合其他 ROS2 的 substitution 类（如 FindPackageShare）动态生成路径，适用于 ROS 2 的 launch 文件。
                        [
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        ]
                    )
                ),
                launch_arguments=[("gz_args", [world, " -r -v ", "3"])],
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_diff_drive_controller],
                )
            ),
            node_robot_state_publisher,
            robot_spawn_entity,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )
