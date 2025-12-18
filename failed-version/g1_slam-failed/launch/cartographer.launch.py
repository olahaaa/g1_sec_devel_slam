import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='g1_cartographer').find('g1_cartographer')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    configuration_basename = LaunchConfiguration('configuration_basename', default='g1_3d.lua')
    rviz_config_dir = os.path.join(pkg_share, 'config')+"/cartographer.rviz"

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        remappings = [
            ('points2', '/utlidar/cloud_livox_mid360'),
            ('imu', '/utlidar/fixed_imu_mid360')]
            )
    
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.1', '-publish_period_sec', '1.0'])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')
    
    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])