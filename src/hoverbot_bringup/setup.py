from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hoverbot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan',
    maintainer_email='your_email@example.com',
    description='HoverBot bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_manager = hoverbot_bringup.lidar_manager:main',
            'test_lidar_control = hoverbot_bringup.test_lidar_control:main',
        ],
    },
)