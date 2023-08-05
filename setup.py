import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'aster_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ushan Fernando',
    maintainer_email='ushanfernando123@gmail.com',
    description='Robot controller for the autonomous mobile robot ASTER',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aster_controller = aster_controller.aster_controller:main',
            'aster_serial = aster_controller.aster_serial:main',
            'aster_odometry = aster_controller.aster_odometry:main',
            'aster_imu = aster_controller.aster_imu:main',
        ],
    },
)
