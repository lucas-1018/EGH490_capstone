from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluerov_waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'matplotlib', 'Pillow'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Terrain-aware coverage waypoint generation + PID follower for BlueROV2 in DAVE',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_publisher = bluerov_waypoint_nav.waypoint_publisher:main',
            'reactive_planner = bluerov_waypoint_nav.reactive_planner:main',
            'path_tracker = bluerov_waypoint_nav.path_tracker:main',
            'pid_controller = bluerov_waypoint_nav.pid_controller:main',
            'vision_pose_bridge = bluerov_waypoint_nav.vision_pose_bridge:main',
            'waypoint_camera_capture = bluerov_waypoint_nav.waypoint_camera_capture:main',
            'sonar_altitude = bluerov_waypoint_nav.sonar_altitude:main',
        ],
    },
)
