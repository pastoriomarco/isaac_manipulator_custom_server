from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'isaac_manipulator_pose_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))
        ),
        (
            os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))
        ),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Pastorio',
    maintainer_email='pastoriomarco@gmail.com',
    description='Service package for multi-object FoundationPose pose extraction in bins with optional per-scan object selection.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'multi_object_pose_server = '
            'isaac_manipulator_pose_server.scripts.multi_object_pose_server:main',
        ],
    },
)
