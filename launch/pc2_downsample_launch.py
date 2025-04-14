# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     """Generate launch description with multiple components."""

#     static_tf_publisher = Node(
#         name="static1",
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         arguments=["0.1", "0.0", "-0.05", "0", "0.785", "0", "base_link", "zed_camera_link"],
#         output="screen"
#     )

#     container = ComposableNodeContainer(
#             name='filter_container',
#             namespace='',
#             package='rclcpp_components',
#             executable='component_container',
#             composable_node_descriptions=[
#                 ComposableNode(
#                     package='pcl_ros',
#                     plugin='pcl_ros::VoxelGrid',
#                     name='voxelgrid',
#                     # remappings=[('/input', '/input_map'),('/output', '/output_map')],
#                     # remappings=[('/input', '/cloud_in'),('/output', '/cloud_in_downsampled')],
#                     remappings=[('/input', '/zed/zed_node/point_cloud/cloud_registered'),('/output', '/cloud_in_downsampled')],
#                     parameters=[{'leaf_size': 0.1}],
#                     # parameters=[{'filter_field_name': 'z', 'filter_limit_min': -0.8, 'filter_limit_max': 2.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
#                 ),
#                 ComposableNode(
#                     package='pcl_ros',
#                     plugin='pcl_ros::VoxelGrid',
#                     name='voxelgrid_ground_removal',
#                     # remappings=[('/input', '/input_map'),('/output', '/output_map')],
#                     remappings=[('/input', '/cloud_transformed'),('/output', '/pc2_filtered')],
#                     # parameters=[{'filter_field_name': 'z', 'filter_limit_min': 0.3, 'filter_limit_max': 3.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
#                     parameters=[{'filter_field_name': 'z', 'filter_limit_min': -0.8, 'filter_limit_max': 2.5, 'filter_limit_negative': False, 'leaf_size': 0.2}],
#                 ),
#             ],
#             output='both',
#     )


#     return LaunchDescription([container, static_tf_publisher])
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

    voxelgrid = Node(
        package='pcl_ros',
        executable='filter_voxel_grid_node',
        name='voxelgrid',
        remappings=[('/input', '/zed/zed_node/point_cloud/cloud_registered'),
                    ('/output', '/cloud_in_downsampled')],
        parameters=[{'leaf_size': 0.2}]
    )

    voxelgrid_ground_removal = Node(
        package='pcl_ros',
        executable='filter_voxel_grid_node',
        name='voxelgrid_ground_removal',
        remappings=[('/input', '/cloud_transformed'),
                    ('/output', '/pc2_filtered')],
        parameters=[{'filter_field_name': 'z', 'filter_limit_min': -0.8, 'filter_limit_max': 2.5, 'filter_limit_negative': False, 'leaf_size': 0.2}]
    )

    return LaunchDescription([static_tf_publisher, voxelgrid, voxelgrid_ground_removal])
