from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'franka_simple_publishers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'std_msgs',
        'franka_msgs'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='fabioamadio93@gmail.com',
    description='A simple ROS 2 package for publishing pose and setting collision behavior for the Franka robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_interactive_marker_pose_publisher = franka_simple_publishers.simple_interactive_marker_pose_publisher:main',
            'interactive_marker_pose_publisher = franka_simple_publishers.interactive_marker_pose_publisher:main',
            'collision_behavior_setter = franka_simple_publishers.collision_behavior_setter:main',
            'assistance_experiment = franka_simple_publishers.assistance_experiment:main',
        ],
    },
)
