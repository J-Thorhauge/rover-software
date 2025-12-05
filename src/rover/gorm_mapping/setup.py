from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'gorm_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aruco_alignment.launch.py']),
        ('share/' + package_name + '/config', ['config/map_alignment.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonas Thorhauge',
    maintainer_email='jonas.thorhauge@gmail.com',
    description='ROS 2 package containging functions and nodes for mapping.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_map_aligner = gorm_mapping.world_map_aligner:main',
            'fake_aruco_tf_publisher = gorm_mapping.fake_aruco_tf_publisher:main',
            'aruco_tf_node = gorm_mapping.aruco_tf_node:main',
        ],
    },
)
