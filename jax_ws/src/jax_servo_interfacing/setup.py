from setuptools import setup, find_packages

package_name = 'jax_servo_interfacing'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/jax_servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Tim Powell',
    maintainer_email='tdp378@yahoo.com',
    description='Servo interfacing for JAX quadruped',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jax_servo_node = jax_servo_interfacing.jax_servo_node:main',
        ],
    },
)
