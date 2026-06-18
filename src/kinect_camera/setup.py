import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kinect_camera'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FlySky',
    maintainer_email='nadeeshajayamanne@protonmail.com',
    description='RGB-D driver node for the Xbox Kinect v1 / Kinect 360 (libfreenect).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_rgbd = kinect_camera.kinect_rgbd:main',
        ],
    },
)
