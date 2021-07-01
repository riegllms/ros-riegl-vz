from setuptools import setup

package_name = 'riconnect'

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
    description='RIEGL low level client service communication library',
    license='RIEGL License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
