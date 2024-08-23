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
                "-0.130625",
                "--y",
                "-1.70488",
                "--z",
                "1.51916",
                "--qx",
                "-0.70126",
                "--qy",
                "0.244657",
                "--qz",
                "0.200819",
                "--qw",
                "0.638787",
                # "--roll",
                # "1.46732",
                # "--pitch",
                # "3.11067",
                # "--yaw",
                # "-2.49811",
            ],
        ),
    ]
    return LaunchDescription(nodes)
