from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'iravath_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Iravath Team',
    maintainer_email='your.email@example.com',
    description='ROS2 perception package for Iravath robot with RealSense camera and object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_camera_node = iravath_perception.realsense_camera_node:main',
            'object_detection_node = iravath_perception.object_detection_node:main',
            'yolo_detection_node = iravath_perception.yolo_detection_node:main',
        ],
    },
)