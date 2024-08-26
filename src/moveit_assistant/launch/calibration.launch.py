""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: wrist3_Link -> aubo_i10/wrist3_Link/camera """
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
                "wrist3_Link",
                "--child-frame-id",
                "aubo_i10/wrist3_Link/camera",
                "--x",
                "0.0016384",
                "--y",
                "-0.011371",
                "--z",
                "0.106924",
                "--qx",
                "3.21155e-05",
                "--qy",
                "0.000702105",
                "--qz",
                "0.999999",
                "--qw",
                "0.00101642",
                # "--roll",
                # "3.14019",
                # "--pitch",
                # "3.14153",
                # "--yaw",
                # "-0.00203279",
            ],
        ),
    ]
    return LaunchDescription(nodes)
