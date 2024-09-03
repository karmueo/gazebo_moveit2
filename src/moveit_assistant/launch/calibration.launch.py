""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: wrist3_Link -> camera_link """
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
                "camera_link",
                "--x",
                "0.00121876",
                "--y",
                "0.00253812",
                "--z",
                "0.112018",
                "--qx",
                "-0.000852297",
                "--qy",
                "-0.00159109",
                "--qz",
                "0.999998",
                "--qw",
                "-0.00120967",
                # "--roll",
                # "0.00318425",
                # "--pitch",
                # "-0.00170074",
                # "--yaw",
                # "-3.13917",
            ],
        ),
    ]
    return LaunchDescription(nodes)
