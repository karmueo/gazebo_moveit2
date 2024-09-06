#!/usr/bin/env -S ros2 launch
"""Visualisation of SDF model for panda in Ignition Gazebo. Note that the generated model://panda/model.sdf descriptor is used."""

from os import path
from typing import List
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="moveit_assistant",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("config", "aubo_i10.urdf.xacro"),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # World and model for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="/home/xvshuo/work/scl/gazebo_moviet2/src/moveit_assistant/worlds/empty.sdf",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    package_name = "moveit_assistant"
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    moveit_config = MoveItConfigsBuilder(
        "aubo_i10", package_name="moveit_assistant"
    ).to_moveit_configs()

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[("gz_args", [world, " -v ", ign_verbosity])],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                moveit_config.robot_description,
                {
                    "publish_frequency": 50.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # ros_gz_sim_create
        Node(
            package="ros_gz_sim",
            executable="create",
            output="log",
            arguments=[
                "-topic",
                "robot_description",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    # 启动控制器
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory(package_name),
                    "config",
                    "ros_gz_bridge_mbot_camera.yaml",
                ),
            }
        ],
    )

    nodes.append(ros_gz_bridge)

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)
