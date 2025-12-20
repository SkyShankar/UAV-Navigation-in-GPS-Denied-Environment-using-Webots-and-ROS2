import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_node_follower_scen2 = Node(
        package='mavic2_ekf_pkg',
        executable='ekf_node_follower_scen2',
        name='ekf_node_follower_scen2'
    )

    logger_node_follower_scen2 = Node(
        package='mavic2_ekf_pkg',
        executable='logger_node_follower_scen2',
        name='logger_node_follower_scen2',
        output='screen'  
    )
    
    return LaunchDescription([
        ekf_node_follower_scen2,
        logger_node_follower_scen2
    ])
