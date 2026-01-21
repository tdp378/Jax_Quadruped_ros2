from setuptools import setup, find_packages 

package_name = 'jax_servo_interfacing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Powell',
    maintainer_email='todo@todo.com',
    description='Servo hardware interface for Jax quadruped',
    license='MIT',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'hardware_interface = jax_servo_interfacing.hardware_interface:main',
        ],
    },
)
