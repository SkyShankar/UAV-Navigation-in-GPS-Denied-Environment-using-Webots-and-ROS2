import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_node_follower_scen1 = Node(
        package='mavic2_ekf_pkg',
        executable='ekf_node_follower_scen1',
        name='ekf_node_follower_scen1'
    )

    logger_node_follower_scen1 = Node(
        package='mavic2_ekf_pkg',
        executable='logger_node_follower_scen1',
        name='logger_node_follower_scen1',
        output='screen'  
    )
    
    return LaunchDescription([
        ekf_node_follower_scen1,
        logger_node_follower_scen1
    ])
