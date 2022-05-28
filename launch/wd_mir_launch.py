from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wd_hga_process',
            executable='wd_mir',
            name='wd_mir'
        )
    ])
