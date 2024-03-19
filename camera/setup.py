from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simon',
    maintainer_email='simontk2010@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection_action = camera.aruco_detection_action:main',
            'aruco_transform_streamer = camera.arcuo_transform_streamer:main',
            'aruco_pose_estimator = camera.aruco_pose_estimator:main'
        ],
    },
)
