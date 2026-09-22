from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    ur_moveit_config = get_package_share_directory(
        'ur_moveit_config'
    )

    kinematics_yaml = os.path.join(
        ur_moveit_config,
        'config',
        'kinematics.yaml'
    )

    draw_letter = Node(
        package='ur3_draw_letter',
        executable='draw_letter_node',
        name='draw_letter',
        output='screen',
        parameters=[
            kinematics_yaml,
            {
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        draw_letter
    ])
