from setuptools import setup

package_name = 'jax_servo_interfacing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/jax_servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tdp378',
    maintainer_email='tdp378@todo.todo',
    description='Servo interface for JAX quadruped',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jax_servo_node = jax_servo_interfacing.jax_servo_node:main',
        ],
    },
)

