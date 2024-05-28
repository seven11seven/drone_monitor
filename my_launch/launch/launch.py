from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='pos_vel_plot',
            arguments=['-perspective-file',
                       ]
        )
    ])


