from setuptools import setup

package_name = 'jax_servo_interfacing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim Powell',
    maintainer_email='todo@todo.com',
    description='Servo hardware interface for Jax quadruped',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [hardware_interface = 
    jax_servo_interfacing.hardware_interface:main',
        ],
    },
)
