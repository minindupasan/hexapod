from setuptools import setup
import os
from glob import glob

package_name = 'hexapod_model_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include mesh files
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include contact files (for reference, but executable will be separate)
        (os.path.join('share', package_name, 'contact'), glob('contact/*')),
        # Include control files
        (os.path.join('share', package_name, 'control'), glob('control/*')),
        # Include gazebo files
        (os.path.join('share', package_name, 'gazebo'), glob('gazebo/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minindu',
    maintainer_email='minindupasan@gmail.com',
    description='Hexapod robot model description with URDF, meshes, and configuration files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contact_sensor = hexapod_model_description.contact_sensor:main',
        ],
    },
)