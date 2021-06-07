from setuptools import setup

package_name = 'py_riegl_vz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Friedl',
    maintainer_email='afriedl@riegl.com',
    description='ROS (Roboter Operating System) Driver for RIEGL VZ scanner series',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'riegl_vz = py_riegl_vz.__init__:main'
        ],
    },
)
