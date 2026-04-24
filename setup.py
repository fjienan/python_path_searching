from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'python_path_searching'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wufy',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner_node = app.astar_planner_node:main',
            'tracker_node = app.tracker_node:main',
            'odom_simulator = app.odom_simulator:main',
            'path_decision_node = app.path_decision_node:main',
        ],
    },
)
