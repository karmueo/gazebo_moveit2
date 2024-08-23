""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> aubo_i10/wrist3_Link/camera """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "aubo_i10/wrist3_Link/camera",
                "--x",
                "-0.0152006",
                "--y",
                "34.1445",
                "--z",
                "1.35012",
                "--qx",
                "-0.528908",
                "--qy",
                "-0.474943",
                "--qz",
                "-0.488221",
                "--qw",
                "0.506287",
                # "--roll",
                # "1.56016",
                # "--pitch",
                # "3.10605",
                # "--yaw",
                # "1.64304",
            ],
        ),
    ]
    return LaunchDescription(nodes)
