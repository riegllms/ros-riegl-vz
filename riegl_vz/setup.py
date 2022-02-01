from glob import glob
from setuptools import setup

package_name = 'riegl_vz'

setup(
    name=package_name,
    version='1.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    ('share/' + package_name, glob('launch/*_launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Friedl',
    maintainer_email='afriedl@riegl.com',
    description='ROS (Roboter Operating System) Driver for RIEGL VZ scanner series',
    license='RIEGL License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'riegl_vz = riegl_vz.__init__:main'
        ],
    },
)
