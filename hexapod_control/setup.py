from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexapod_control'

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
    maintainer='minindu',
    maintainer_email='minindu@todo.todo',
    description='Hexapod locomotion control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = hexapod_control.joint_controller:main',
            'walking_controller = hexapod_control.walking_controller:main',
        ],
    },
)
