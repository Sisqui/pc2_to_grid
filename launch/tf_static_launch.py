from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_publisher = Node(
        name="static1",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.1", "0.0", "-0.05", "0", "0.785", "0", "base_link", "zed_camera_link"],
        output="screen"
    )

    return LaunchDescription([static_tf_publisher])
