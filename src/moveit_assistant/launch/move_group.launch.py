from moveit_configs_utils import MoveItConfigsBuilder

# from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml


def load_yaml(package_path, file_path):
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_move_group_launch(moveit_config):
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    # 加载非默认 MoveGroup 功能（空格分隔）
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            # default_value=moveit_config.move_group_capabilities["capabilities"]
            default_value="",
        )
    )
    # 禁止这些默认的 MoveGroup 功能（空格分隔）
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # 不要将动态信息从 /joint_states 复制到内部机器人监控
    # 默认为 false，因为 move_group 中几乎没有任何内容依赖于此信息
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: 必须将以下值包装起来，以便参数值可以为空字符串
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # 发布物理机器人的规划场景，以便rviz插件了真实的机器人
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        # TODO:仿真时这里要改为True
        "use_sim_time": True,
    }

    # 规划器
    ompl_planning_pipeline_config = {
        "move_group": {
            "sample_duration": 0.005,
        }
    }
    ompl_planning_yaml = load_yaml(
        moveit_config.package_path, "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        ompl_planning_pipeline_config,
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ["DISPLAY"]},
    )
    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "aubo_i10", package_name="moveit_assistant"
    ).to_moveit_configs()
    return generate_move_group_launch(moveit_config)
