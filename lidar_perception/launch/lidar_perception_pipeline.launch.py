import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_perception')
    params_file = os.path.join(pkg_dir, 'config', 'lidar_perception_params.yaml')

    # Since we don't have the params file written yet, we can pass parameters directly or via file.
    # For now, we'll use an empty default and let nodes use their C++ defaults.
    
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_perception',
                plugin='lidar_perception::RoiCropFilterNode',
                name='roi_crop_filter_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='lidar_perception',
                plugin='lidar_perception::VoxelGridFilterNode',
                name='voxel_grid_filter_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='lidar_perception',
                plugin='lidar_perception::GroundRemovalNode',
                name='ground_removal_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='lidar_perception',
                plugin='lidar_perception::EuclideanClusterNode',
                name='euclidean_cluster_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='lidar_perception',
                plugin='lidar_perception::PipelineMonitorNode',
                name='pipeline_monitor_node',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
