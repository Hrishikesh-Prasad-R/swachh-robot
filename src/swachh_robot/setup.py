from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'swachh_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bmscecse',
    maintainer_email='bmscecse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonomous_navigator = swachh_robot.autonomous_navigator:main',
            'box_marker = swachh_robot.box_marker:main',
            'arrow_teleop = swachh_robot.arrow_teleop:main',
            'environment_viz = swachh_robot.environment_viz:main',
            'sensor_validator = swachh_robot.sensor_validator:main',
            'slam_readiness = swachh_robot.slam_readiness:main',
        ],
    },
)
