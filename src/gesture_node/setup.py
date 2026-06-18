import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gesture_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ONNX hand-landmark models (downloaded by setup_qbot_env.sh)
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FlySky',
    maintainer_email='nadeeshajayamanne@protonmail.com',
    description='Hand-gesture recognition from the Kinect RGB feed.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_command_node = gesture_node.gesture_command_node:main',
        ],
    },
)
