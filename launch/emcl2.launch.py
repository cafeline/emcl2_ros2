import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cutter_params_file = LaunchConfiguration('cutter_params_file')
    cutter_regions_file = LaunchConfiguration('cutter_regions_file')
    vq_params_file = LaunchConfiguration('vq_params_file')
    vq_map_file = LaunchConfiguration('vq_map_file')
    simple_map_file = LaunchConfiguration('simple_map_file')
    obstacle_tracker_params_file = LaunchConfiguration('obstacle_tracker_params_file')
    tvvf_vo_params_file = LaunchConfiguration('tvvf_vo_params_file')
    waypoint_params_file = LaunchConfiguration('waypoint_params_file')
    waypoint_csv_file = LaunchConfiguration('waypoint_csv_file')
    auto_start = LaunchConfiguration('auto_start')
    rviz_enable = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=[TextSubstitution(text=os.path.join(get_package_share_directory('emcl2'), 'config', '')),
            TextSubstitution(text='emcl2_with_compressed_map.param.yaml')],
        description='emcl2 param file path')
    declare_cutter_params = DeclareLaunchArgument(
        'cutter_params_file',
        default_value=os.path.join(get_package_share_directory('pointcloud2_cutter'),
            'config', 'pointcloud2_cutter.param.yaml'),
        description='pointcloud2_cutter param file path')
    declare_cutter_regions = DeclareLaunchArgument(
        'cutter_regions_file',
        default_value=os.path.join(
            get_package_share_directory('pointcloud2_cutter'),
            'config',
            'tsudanuma_regions.yaml'),
        description='pointcloud2_cutter regions file path')
    declare_vq_params = DeclareLaunchArgument(
        'vq_params_file',
        default_value=os.path.join(
            get_package_share_directory('vq_server'),
            'config',
            'vq_server.params.yaml'),
        description='vq_server param file path')
    declare_vq_map = DeclareLaunchArgument(
        'vq_map_file',
        default_value=os.path.join(
            get_package_share_directory('vq_server'),
            'maps',
            'tsudanuma_voxelsize_05_compressed_map.h5'),
        description='Compressed voxel map for vq_server')
    declare_simple_map_file = DeclareLaunchArgument(
        'simple_map_file',
        default_value=os.path.join(
            get_package_share_directory('raspicat_tvvf_navigation'),
            'maps',
            'nav.yaml'),
        description='Map yaml file for simple_map_server')
    declare_obstacle_tracker_params = DeclareLaunchArgument(
        'obstacle_tracker_params_file',
        default_value=os.path.join(
            get_package_share_directory('raspicat_tvvf_navigation'),
            'config',
            'obstacle_tracker_params.yaml'),
        description='Obstacle tracker param file path')
    declare_tvvf_vo_params = DeclareLaunchArgument(
        'tvvf_vo_params_file',
        default_value=os.path.join(
            get_package_share_directory('raspicat_tvvf_navigation'),
            'config',
            'tvvf_vo_params.yaml'),
        description='tvvf_vo_c param file path')
    declare_waypoint_params = DeclareLaunchArgument(
        'waypoint_params_file',
        default_value=os.path.join(
            get_package_share_directory('raspicat_tvvf_navigation'),
            'config',
            'waypoint_follower_params.yaml'),
        description='Waypoint follower param file path')
    declare_waypoint_csv = DeclareLaunchArgument(
        'waypoint_csv_file',
        default_value=os.path.join(
            get_package_share_directory('raspicat_tvvf_navigation'),
            'maps',
            'tsudanuma_WP.csv'),
        description='Waypoint CSV file path')
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start waypoint navigation')
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('raspicat_tvvf_navigation'),
            'rviz',
            'navigation.rviz'),
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
                parameters=[
                    cutter_params_file,
                    {'regions_config_path': cutter_regions_file}
                ],
                output='screen'),
            Node(
                package='vq_server',
                executable='vq_server',
                name='vq_server',
                parameters=[
                    vq_params_file,
                    {'map_file': vq_map_file}
                ],
                output='screen'),
            Node(
                package='raspicat_tvvf_navigation',
                executable='simple_map_server',
                name='map_server',
                parameters=[{'map_yaml_path': simple_map_file}],
                output='screen'),
            Node(
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[params_file],
                output='screen'),
            Node(
                package='obstacle_tracker',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                parameters=[obstacle_tracker_params_file],
                output='screen'),
            Node(
                package='tvvf_vo_c',
                executable='tvvf_vo_c_node',
                name='tvvf_vo_c_node',
                parameters=[tvvf_vo_params_file],
                output='screen'),
            Node(
                package='raspicat_tvvf_navigation',
                executable='waypoint_follower_node',
                name='waypoint_follower_node',
                parameters=[
                    waypoint_params_file,
                    {
                        'waypoint_csv_path': waypoint_csv_file,
                        'auto_start': auto_start,
                    }
                ],
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
    ld.add_action(declare_params_file)
    ld.add_action(declare_cutter_params)
    ld.add_action(declare_cutter_regions)
    ld.add_action(declare_vq_params)
    ld.add_action(declare_vq_map)
    ld.add_action(declare_simple_map_file)
    ld.add_action(declare_obstacle_tracker_params)
    ld.add_action(declare_tvvf_vo_params)
    ld.add_action(declare_waypoint_params)
    ld.add_action(declare_waypoint_csv)
    ld.add_action(declare_auto_start)
    ld.add_action(declare_rviz)
    ld.add_action(declare_rviz_config)

    ld.add_action(launch_node)

    return ld
