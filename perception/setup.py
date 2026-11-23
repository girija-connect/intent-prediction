import os
from glob import glob
from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Human perception: YOLOv8 + ByteTrack + lidar range fusion + Kalman filter -> /human_tracks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_tracker = perception.human_tracker:main',
        ],
    },
)
