from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'jax_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install URDF / xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # Install mesh files
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),

        # Install RViz configs
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Install Controllers configs
        (os.path.join('share', package_name, 'config'),
             glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tdp378',
    maintainer_email='tdp378@yahoo.com',
    description='Jax quadruped robot description',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
)
