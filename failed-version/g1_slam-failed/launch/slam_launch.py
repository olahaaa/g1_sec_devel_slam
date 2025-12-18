import os
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='g1_cartographer').find('g1_cartographer')

    urdf_file = os.path.join(pkg_share, 'urdf', 'g1.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'cartographer.rviz')

    configuration_directory = os.path.join(pkg_share, 'config')  
    configuration_basename = 'g1_3d.lua'                         


    try:
        with open(urdf_file, 'r') as infp:
            robot_description = infp.read()
    except EnvironmentError as e:
        print(f"⚠️ Failed to open URDF file: {urdf_file}")
        raise e

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description,
        }]
    )

    imu_fix_node = Node(
        package='fix_imu',
        executable='imu_fix_node',
        name='imu_fix_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    cloud_fix_node = Node(
        package='fix_imu',
        executable='cloud_fix_node',
        name='cloud_fix_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', configuration_directory, 
            '-configuration_basename', configuration_basename     
        ],
        remappings=[
            ('imu', '/utlidar/fixed_imu_mid360'),
            ('points2', '/utlidar/cloud_livox_mid360')
        ]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.1', '-publish_period_sec', '1.0']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        imu_fix_node,
        cloud_fix_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])