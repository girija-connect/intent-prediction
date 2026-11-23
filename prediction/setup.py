import os
from glob import glob
from setuptools import setup

package_name = 'prediction'

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
    description='Human intent prediction (CPA classifier) feeding the Nav2 costmap / speed limit; legacy stand-alone controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intent_nav_bridge = prediction.intent_nav_bridge:main',
            'intent_controller = prediction.intent_controller:main',
        ],
    },
)
