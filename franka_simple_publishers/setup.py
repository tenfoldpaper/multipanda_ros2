from setuptools import find_packages, setup

package_name = 'franka_simple_publishers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jin-mirmi',
    maintainer_email='s.bien@tum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_marker_pose_publisher = franka_simple_publishers.interactive_marker_pose_publisher:main',
            'collision_behavior_setter = franka_simple_publishers.collision_behavior_setter:main',
        ],
    },
)
