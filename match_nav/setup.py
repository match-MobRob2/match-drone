from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'match_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        # Include docs
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='luca0204@freenet.de',
    description='3D Navigation Stack for Match Drone with Octomap, OMPL Path Planning and Autonomous Exploration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'global_planner = match_nav.global_planner:main',
            'waypoint_follower = match_nav.waypoint_follower:main',
            'collision_checker = match_nav.collision_checker:main',
            'frontier_explorer = match_nav.frontier_explorer:main',
            'mission_manager = match_nav.mission_manager:main',
        ],
    },
)
