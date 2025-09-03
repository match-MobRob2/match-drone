from setuptools import find_packages, setup

package_name = 'match_control'

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
    maintainer='luca',
    maintainer_email='Luca0204@freenet.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_flight = match_control.simple_flight:main',
            'teleop_driven_flight = match_control.teleop_driven_flight:main',
            'drone_services = match_control.drone_services:main',
            'demo_takeoff_land = match_control.demo_takeoff_land:main',
            'demo_takeoff_forward_land = match_control.demo_takeoff_forward_land:main',
            'demo_takeoff_square_land = match_control.demo_takeoff_square_land:main',
            'demo_takeoff_circle_land = match_control.demo_takeoff_circle_land:main',
            'exercise_takeoff_land = match_control.exercise_takeoff_land:main',
            'exercise_takeoff_forward_land = match_control.exercise_takeoff_forward_land:main',
            'exercise_takeoff_square_land = match_control.exercise_takeoff_square_land:main',
            'exercise_takeoff_circle_land = match_control.exercise_takeoff_circle_land:main',
        ],
    },
)
