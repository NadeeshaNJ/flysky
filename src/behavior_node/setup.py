import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'behavior_node'

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
    description='Pet-like behavior controller fusing vision and gesture inputs.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pet_behavior_node = behavior_node.pet_behavior_node:main',
        ],
    },
)
