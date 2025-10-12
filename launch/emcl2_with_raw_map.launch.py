import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('emcl2')
    default_params = os.path.join(
        package_dir,
        'config',
        'emcl2_with_raw_map.param.yaml'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the YAML file with parameters for emcl2_node'
    )

    emcl2_node = Node(
        package='emcl2',
        executable='emcl2_node',
        name='emcl2',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_params)
    ld.add_action(emcl2_node)
    return ld
