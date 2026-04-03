from setuptools import find_packages, setup

package_name = 'match_utils'

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
    maintainer_email='luca0204@freenet.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mavros_local_to_tf = match_utils.mavros_local_to_tf:main',
            'imu_timemachine = match_utils.imu_timemachine:main',
            'cloud_nan_filter = match_utils.cloud_nan_filter:main',
            'pointcloud_to_livox = match_utils.pointcloud_to_livox:main',
            'odometry_to_drone = match_utils.odometry_to_drone:main',
            'lidar_camera_fusion = match_utils.map_livox_to_realsense:main',
            'map_livox_to_realsense = match_utils.map_livox_to_realsense:main',
        ],
    },
)
