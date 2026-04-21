from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_bird_eye_view_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Bird eye view controller for TurtleBot3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bev_controller = turtlebot3_bird_eye_view_control.turtlebot3_bird_eye_view_control:main',
        ],
    },
)
