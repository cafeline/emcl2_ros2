import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    nav_params_file = LaunchConfiguration('nav_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_enable = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_nav_params = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(
            get_package_share_directory('emcl2'),
            'config',
            'tsukuba.yaml'),
        description='Unified parameter file for navigation stack')
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('emcl2'),
            'rviz2',
            'emcl2.rviz'),
        description='RViz configuration file path')


    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='livox_static_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
                output='screen'),
            Node(
                package='pointcloud2_cutter',
                executable='pointcloud2_cutter_node',
                name='pointcloud2_cutter',
                parameters=[nav_params_file],
                output='screen'),
            Node(
                package='vq_server',
                executable='vq_server',
                name='vq_server',
                parameters=[
                    nav_params_file
                ],
                output='screen'),
            Node(
                package='raspicat_tvvf_navigation',
                executable='simple_map_server',
                name='map_server',
                parameters=[nav_params_file],
                output='screen'),
            Node(
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[nav_params_file],
                output='screen'),
            Node(
                package='obstacle_tracker',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                parameters=[nav_params_file],
                output='screen'),
            Node(
                package='tvvf_vo_c',
                executable='tvvf_vo_c_node',
                name='tvvf_vo_c_node',
                parameters=[nav_params_file],
                output='screen'),
            Node(
                package='raspicat_tvvf_navigation',
                executable='waypoint_follower_node',
                name='waypoint_follower_node',
                parameters=[nav_params_file],
                output='screen'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                condition=IfCondition(rviz_enable)
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav_params)
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_config)

    ld.add_action(launch_node)

    return ld
