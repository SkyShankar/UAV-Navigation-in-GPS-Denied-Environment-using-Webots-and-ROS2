import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_node = Node(
        package='mavic2_ekf_pkg',
        executable='ekf_node',
        name='ekf_node'
    )

    logger_node_ekf = Node(
        package='mavic2_ekf_pkg',
        executable='logger_node_ekf',
        name='logger_node_ekf',
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
        ekf_node,
        logger_node_ekf,
        keyboard_controller_node
    ])
