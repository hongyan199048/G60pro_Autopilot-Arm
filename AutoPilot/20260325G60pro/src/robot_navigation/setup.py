from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_sim.launch.py', 'launch/navigation_real.launch.py', 'launch/nav2_no_amcl.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params_sim.yaml', 'config/nav2_params_real.yaml']),
        # maps 目录已移至顶层工作空间 maps/，不再打包到 share
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin123',
    maintainer_email='admin123@todo.todo',
    description='G60Pro navigation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'initial_pose_relay = robot_navigation.initial_pose_relay:main',
        ],
    },
)