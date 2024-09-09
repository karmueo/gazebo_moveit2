""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """

""" EYE-IN-HAND: wrist3_Link -> right_camera_link """
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
                "right_camera_link_calibrated",
                "--x",
                "0.098489",
                "--y",
                "-0.0202836",
                "--z",
                "0.122815",
                "--qx",
                "-0.00570172",
                "--qy",
                "-0.00134068",
                "--qz",
                "0.999982",
                "--qw",
                "0.000881187",
                # "--roll",
                # "0.00267145",
                # "--pitch",
                # "-0.0114058",
                # "--yaw",
                # "3.13985",
            ],
        ),
    ]
    return LaunchDescription(nodes)
