import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sdre_node = Node(
        package='mavic2_ekf_pkg',
        executable='sdre_node', 
        name='sdre_node'
    )

    logger_node_sdre = Node(
        package='mavic2_ekf_pkg',
        executable='logger_node_sdre', 
        name='logger_node_sdre',
        output='screen'  
    )

    keyboard_controller_node = Node(
        package='mavic2_ekf_pkg',
        executable='key_board_controller',  
        name='key_board_controller',
        prefix='xterm -e', 
        output='screen'
    )

    return LaunchDescription([
        sdre_node,
        logger_node_sdre,
        keyboard_controller_node
    ])
