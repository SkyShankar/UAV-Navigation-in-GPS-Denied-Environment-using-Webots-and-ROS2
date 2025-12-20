import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_node = Node(
        package='mavic2_ekf_pkg',
        executable='ekf_node',
        name='ekf_node'
    )

    logger_node_leader_scen1 = Node(
        package='mavic2_ekf_pkg',
        executable='logger_node_leader_scen1',
        name='logger_node_leader_scen1',
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
        logger_node_leader_scen1,
        keyboard_controller_node
    ])
