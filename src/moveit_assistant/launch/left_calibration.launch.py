""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """

""" EYE-IN-HAND: wrist3_Link -> left_camera_link """
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
                "left_camera_link_calibrated",
                "--x",
                "-0.10645",
                "--y",
                "0.0164793",
                "--z",
                "0.172569",
                "--qx",
                "0.00391929",
                "--qy",
                "0.00258406",
                "--qz",
                "0.999776",
                "--qw",
                "-0.0206281",
                # "--roll",
                # "3.13626",
                # "--pitch",
                # "3.13386",
                # "--yaw",
                # "0.0412802",
            ],
        ),
    ]
    return LaunchDescription(nodes)
