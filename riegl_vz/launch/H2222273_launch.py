from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='riegl_vz',
            executable='riegl_vz',
            name='riegl_vz_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'hostname': '192.168.3.101'},
                {'working_dir': '/home/af/.ros_riegl_vz'},
                {'ssh_user': 'user'},
                {'ssh_password': 'user'},
                {'stor_media': 2},
                {'project_name': ''},
                {'scan_publish': True},
                {'scan_register': False},
                {'scan_pattern': [30.0,130.0,0.1,0.0,10.0,0.1]},
                {'scan_filter': ''},
                {'meas_program': 3},
                {'msm': 1}
            ],
            arguments = ['--ros-args', '--log-level', 'DEBUG'],
        )
    ])
