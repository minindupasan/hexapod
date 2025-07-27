from setuptools import setup
import os
from glob import glob

package_name = 'hexapod_model_description'

def get_data_files(package_name):
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    for subdir in ['launch', 'urdf', 'meshes', 'config', 'control', 'gazebo', 'scripts']:
        files = glob(f'{subdir}/*')
        if files:
            data_files.append((os.path.join('share', package_name, subdir), files))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minindu',
    maintainer_email='minindupasan@gmail.com',
    description='Hexapod robot model description with URDF, meshes, and configuration files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)