from setuptools import find_packages, setup

package_name = 'sim_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','open3d','pandas'],
    zip_safe=True,
    maintainer='daghbeji',
    maintainer_email='abderraouf7@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # frame names misalignment fixes
            'pc_frame_fix = sim_utils.pc_frame_fix:main',
            'imu_frame_fix = sim_utils.imu_frame_fix:main',
            'odom_bridge = sim_utils.odom_bridge:main',
            # convert SDF to 3D-pointcloud
            'sdf_to_pcl_converter = sim_utils.sdf_to_pcl:main',
            # test fast-lio with New college dataset
            'odom_tf = sim_utils.odom_tf:main',
            'pcd_replay = sim_utils.pcd_replay:main',
            'gt_map_pub = sim_utils.gt_map_pub:main',
        ],
    },
)
