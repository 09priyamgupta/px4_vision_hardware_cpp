from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'px4_vision_hardware_cpp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config',
            glob(os.path.join('config', '*.yaml'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='priyam22',
    maintainer_email='u1999097@campus.udg.edu',
    description='AprilTag-based precision landing for PX4 using vision feedback',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
        'console_scripts': [
            'gps_rover_enu_publisher = px4_vision_hardware_cpp.gps_rover_enu_publisher:main',
            'landing_director = px4_vision_hardware_cpp.landing_director:main',
            'apriltag_relative_pose = px4_vision_hardware_cpp.apriltag_relative_pose:main',
            'tf_state_logger = px4_vision_hardware_cpp.tf_state_logger:main',
            'tag_rviz_markers = px4_vision_hardware_cpp.tag_rviz_markers:main',
            'rover_relative_state_node = px4_vision_hardware_cpp.rover_relative_state_node:main',
            'rover_state_ekf_node = px4_vision_hardware_cpp.rover_state_ekf_node:main',
        ],
    },
)
