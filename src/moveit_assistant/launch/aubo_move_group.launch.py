import yaml
from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

"""
新用assistant配置一个模型之后要把ompl_planning.yaml文件放置到config文件夹中,然后planning_pipeline和planning_scene_monitor_parameters要添加到move_group之中
同时move要设置use_sim_time,rviz不要设置use_sim_time
"""


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    package_name = "moveit_assistant"
    moveit_config = (
        MoveItConfigsBuilder("aubo_i10", package_name="moveit_assistant")
        .robot_description(file_path="config/aubo_i10.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory(package_name)
        + "/config/moveit.rviz"  # 没有move_group.rviz，有moveit.rviz
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            rviz_node,
            run_move_group_node,
        ]
    )
