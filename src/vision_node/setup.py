import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FlySky',
    maintainer_email='nadeeshajayamanne@protonmail.com',
    description='Face detection and tracking from the Kinect RGB feed using OpenCV.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_tracker_node = vision_node.face_tracker_node:main',
        ],
    },
)
