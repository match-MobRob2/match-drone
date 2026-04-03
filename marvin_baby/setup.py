from setuptools import find_packages, setup

package_name = 'marvin_baby'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/baby-marvin.launch.py']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/marvin-baby.xacro']),
        ('share/' + package_name + '/urdf/mesh', ['urdf/mesh/Baby-Marvin.glb']),
        ('share/' + package_name + '/urdf/mesh', ['urdf/mesh/Baby-Marvin-Rad.glb']),
        ('share/' + package_name + '/rviz', ['rviz/robot.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/mecanum.sdf']),

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
        ],
    },
)
