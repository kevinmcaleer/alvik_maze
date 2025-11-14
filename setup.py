from setuptools import setup
import os
from glob import glob

package_name = 'alvik_mapping'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kev',
    maintainer_email='kev@kevsrobots.com',
    description='Arduino Alvik mapping and navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_receiver = alvik_mapping.tof_receiver:main',
            'room_mapper = alvik_mapping.room_mapper:main',
            'goal_commander = alvik_mapping.goal_commander:main',
            'alvik_bridge = alvik_mapping.alvik_bridge:main',
        ],
    },
)
