import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hoverbot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='jrnovosad@gmail.com',
    description='Hoverboard motor driver for HoverBot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hoverbot_driver_node = hoverbot_driver.hoverbot_driver_node:main',
        ],
    },
)
