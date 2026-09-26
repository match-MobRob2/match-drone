import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'marvin_ui'


def tree(src):
    """Alle Dateien unter src als data_files (Unterordner bleiben erhalten)."""
    out = {}
    for f in glob(os.path.join(src, '**', '*'), recursive=True):
        if os.path.isfile(f):
            out.setdefault(os.path.join('share', package_name, os.path.dirname(f)), []).append(f)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + tree('web'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='luca0204@freenet.de',
    description='Marvin Ground Control (Web-UI + Supervisor)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'supervisor = marvin_ui.supervisor:main',
            'fake_gcs = marvin_ui.fake_gcs:main',
        ],
    },
)
