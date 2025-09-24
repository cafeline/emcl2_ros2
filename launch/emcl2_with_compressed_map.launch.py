import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('emcl2')

    default_map = os.path.join(package_dir, 'map', 'compressed_map.h5')

    declare_map = DeclareLaunchArgument(
        'map_hdf5_path',
        default_value=default_map,
        description='Path to the compressed HDF5 map file'
    )

    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/pointcloud',
        description='PointCloud2 topic to subscribe'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    emcl2_node = Node(
        package='emcl2',
        executable='emcl2_node',
        name='emcl2',
        parameters=[
            {
                'map_hdf5_path': LaunchConfiguration('map_hdf5_path'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_pointcloud_topic)
    ld.add_action(declare_use_sim_time)
    ld.add_action(emcl2_node)
    return ld
