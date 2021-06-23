import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = os.path.join(
        get_package_share_directory('riegl_vz'),
        'config',
        'params.yaml'
    )

    node = Node(
        package='riegl_vz',
        executable='riegl_vz',
        name='riegl_vz_node',
        output='screen',
        emulate_tty=True,
        parameters=[params],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    ld.add_action(node)
    return ld
