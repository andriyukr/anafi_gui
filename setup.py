#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

package_name = 'anafi_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('config/*.qss')),
        (os.path.join('share', package_name), glob('icon/*.png'))
    ],
    install_requires=['setuptools'],  # pip install setuptools==58.2.0
    zip_safe=True,
    maintainer='andriy',
    maintainer_email='andriy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anafi_gui = anafi_gui.gui_node:main',
            'image_display=anafi_gui.test:main',
        ],
    },
)
