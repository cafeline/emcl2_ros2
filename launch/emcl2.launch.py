import copy
import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_emcl2 = get_package_share_directory('emcl2')
    pkg_pointcloud2 = get_package_share_directory('pointcloud2_cutter')
    pkg_raspicat_nav = get_package_share_directory('raspicat_tvvf_navigation')
    pkg_vq_server = get_package_share_directory('vq_server')
    nav_params_path = os.path.join(pkg_emcl2, 'config', 'tsukuba.yaml')

    with open(nav_params_path, 'r', encoding='utf-8') as f:
        nav_params = yaml.safe_load(f)

    default_use_rviz = (
        'true' if nav_params.get('/**', {}).get('ros__parameters', {}).get('use_rviz', True)
        else 'false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    map_hdf5_file = LaunchConfiguration('map_hdf5_file')
    regions_config_file = LaunchConfiguration('regions_config_file')
    waypoint_csv_file = LaunchConfiguration('waypoint_csv_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    vq_map_file = LaunchConfiguration('vq_map_file')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value=default_use_rviz)

    declare_rviz_config = DeclareLaunchArgument('rviz_config_file',
        default_value=os.path.join(pkg_emcl2, 'rviz2', 'emcl2.rviz'))

    declare_map_hdf5 = DeclareLaunchArgument('map_hdf5_file',
        default_value=os.path.join(pkg_emcl2, 'maps', 'tsukuba_adjusted_fusion_12061418_cityhall_-1.h5'))

    declare_regions_config = DeclareLaunchArgument('regions_config_file',
        default_value=os.path.join(pkg_pointcloud2, 'config', 'tsukuba_regions.yaml'))

    declare_waypoint_csv = DeclareLaunchArgument('waypoint_csv_file',
        default_value=os.path.join(pkg_raspicat_nav, 'maps', 'tsukuba_WP.csv'))

    declare_map_yaml = DeclareLaunchArgument('map_yaml_file',
        default_value=os.path.join(pkg_raspicat_nav, 'maps', 'navigation_map.yaml'))

    declare_vq_map = DeclareLaunchArgument('vq_map_file',
        default_value=os.path.join(pkg_vq_server,'maps', 'tsukuba_adjusted_fusion_cityhall_-1_view.h5'))


    def launch_setup(context, *args, **kwargs):
        def to_bool(value):
            resolved = value
            if isinstance(resolved, LaunchConfiguration):
                resolved = context.perform_substitution(resolved)
            if isinstance(resolved, str):
                return resolved.lower() in ('1', 'true', 'yes', 'on')
            return bool(resolved)

        def resolved_value(value):
            if isinstance(value, LaunchConfiguration):
                return context.perform_substitution(value)
            return value

        merged_params = copy.deepcopy(nav_params)
        merged_params.setdefault('emcl2', {}).setdefault('ros__parameters', {})[
            'map_hdf5_path'
        ] = resolved_value(map_hdf5_file)
        merged_params.setdefault('pointcloud2_cutter', {}).setdefault('ros__parameters', {})[
            'regions_config_path'
        ] = resolved_value(regions_config_file)
        merged_params.setdefault('map_server', {}).setdefault('ros__parameters', {})[
            'map_yaml_path'
        ] = resolved_value(map_yaml_file)
        merged_params.setdefault('waypoint_follower_node', {}).setdefault('ros__parameters', {})[
            'waypoint_csv_path'
        ] = resolved_value(waypoint_csv_file)
        merged_params.setdefault('vq_server', {}).setdefault('ros__parameters', {})[
            'map_file'
        ] = resolved_value(vq_map_file)
        merged_params.setdefault('/**', {}).setdefault('ros__parameters', {})[
            'use_rviz'
        ] = to_bool(use_rviz)

        tmp = tempfile.NamedTemporaryFile(
            mode='w', delete=False, prefix='emcl2_nav_', suffix='.yaml'
        )
        yaml.safe_dump(merged_params, tmp, allow_unicode=True)
        params_file = tmp.name
        tmp.close()

        return [
            GroupAction(
                actions=[
                    SetParameter('use_sim_time', resolved_value(use_sim_time)),
                    Node(
                        package='pointcloud2_cutter',
                        executable='pointcloud2_cutter_node',
                        name='pointcloud2_cutter',
                        parameters=[params_file],
                        output='screen'),
                    Node(
                        package='vq_server',
                        executable='vq_server',
                        name='vq_server',
                        parameters=[params_file],
                        output='screen',
                        condition=IfCondition(use_rviz)),
                    Node(
                        package='raspicat_tvvf_navigation',
                        executable='simple_map_server',
                        name='map_server',
                        parameters=[params_file],
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
                        parameters=[params_file,  {'use_sim_time': False}],
                        output='screen'),
                    Node(
                        package='tvvf_vo_c',
                        executable='tvvf_vo_c_node',
                        name='tvvf_vo_c_node',
                        parameters=[params_file],
                        output='screen'),
                    Node(
                        package='velocity_smoother',
                        executable='velocity_smoother',
                        name='velocity_smoother',
                        parameters=[params_file],
                        output='screen'),
                    Node(
                        package='raspicat_tvvf_navigation',
                        executable='waypoint_follower_node',
                        name='waypoint_follower_node',
                        parameters=[params_file],
                        output='screen'),
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        arguments=['-d', rviz_config_file],
                        condition=IfCondition(use_rviz)
                    ),
                    Node(
                        package='imu_rpy_pose',
                        executable='imu_rpy_pose_node',
                        name='imu_rpy_pose',
                        parameters=[params_file],
                        output='screen'),
                ]
            )
        ]

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_map_hdf5)
    ld.add_action(declare_regions_config)
    ld.add_action(declare_waypoint_csv)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_vq_map)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
