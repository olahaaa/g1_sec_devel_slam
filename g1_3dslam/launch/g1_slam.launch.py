import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='g1_3dslam').find('g1_3dslam')
    config_directory = os.path.join(pkg_share,'config')
    config_basename = 'backpack_3d.lua'
    urdf_file = os.path.join(pkg_share, 'urdf', 'g1.urdf')

    robo_state_pulisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': False}]
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', config_directory,
            '-configuration_basename', config_basename
        ],
        remappings=[
            ('imu', '/livox/imu'),
            ('points2', '/livox/lidar') 
        ]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['resolution', '0.1', '-publisher_period_sec','1.0']
    )

    return LaunchDescription([
        robo_state_pulisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node
    ])