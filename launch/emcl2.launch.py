import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    hardcoded_map_yaml = '/home/ryo/raspicat_ws/src/maps/mile1_2_50.yaml'
    use_sim_time = LaunchConfiguration('use_sim_time')
    cutter_params_file = LaunchConfiguration('cutter_params_file')
    cutter_regions_file = LaunchConfiguration('cutter_regions_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('emcl2'), 'config', '')),
            TextSubstitution(text='emcl2_with_voxelhash_map.param.yaml')],
        description='emcl2 param file path')
    declare_cutter_params = DeclareLaunchArgument(
        'cutter_params_file',
        default_value=os.path.join(
            get_package_share_directory('pointcloud2_cutter'),
            'config',
            'pointcloud2_cutter.param.yaml'),
        description='pointcloud2_cutter param file path')
    declare_cutter_regions = DeclareLaunchArgument(
        'cutter_regions_file',
        default_value=os.path.join(
            get_package_share_directory('pointcloud2_cutter'),
            'config',
            'tsudanuma_regions.yaml'),
        description='pointcloud2_cutter regions file path')

    lifecycle_nodes = ['map_server']

    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{'yaml_filename': hardcoded_map_yaml}],
                output='screen'),
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
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[params_file],
                output='screen'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_cutter_params)
    ld.add_action(declare_cutter_regions)

    ld.add_action(launch_node)

    return ld
