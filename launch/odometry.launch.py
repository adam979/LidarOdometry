import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments 
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    # Paths
    pkg_share  = FindPackageShare('lidar_odometry')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'odometry.rviz'])

    # Odometry node
    odometry_node = Node(
        package='lidar_odometry',
        executable='odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[params_file]
    )

    # RViz2 node (only launched if rviz:=true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        odometry_node,
        rviz_node,
    ])