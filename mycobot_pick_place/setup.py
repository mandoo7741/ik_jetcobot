from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mycobot_pick_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bjh',
    maintainer_email='bjhee0430@gmail.com',
    description='Pick and Place package for MyCobot with AprilTag',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_node = pick_place_node:main',
        ],
    },
)
