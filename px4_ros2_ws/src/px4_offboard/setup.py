from setuptools import setup
import os
from glob import glob

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Madhvendra',
    maintainer_email='madhavyadav811@gmail.com',
    description='PX4 Offboard Control with Teleop',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard.py = px4_offboard.teleop_keyboard:main',
            'velocity_offboard_control.py = px4_offboard.velocity_offboard_control:main', 
        ],
    },
)
