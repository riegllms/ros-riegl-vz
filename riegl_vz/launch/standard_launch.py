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
                {'hostname': 'H2222222'},
                {'working_dir': '~/.ros_riegl_vz'},
                {'ssh_user': 'user'},
                {'ssh_password': 'user'},
                {'stor_media': 2},
                {'project_name': ''},
                {'scan_publish': True},
                {'scan_register': False},
                {'scan_pattern': [30.0,130.0,0.04,0.0,360.0,0.04]},
                {'meas_program': 3},
                {'msm': 1}
            ]
        )
    ])
