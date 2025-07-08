from setuptools import find_packages, setup

package_name = 'match_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gz.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gz2.launch.py']),
        ('share/' + package_name + '/launch', ['launch/x500.launch.py']),
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
        ],
    },
)
