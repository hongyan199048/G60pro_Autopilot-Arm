from setuptools import setup
import os
from glob import glob

package_name = 'robot_bringup'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# 添加所有 launch 文件
launch_files = glob('launch/*.launch.py')
for f in launch_files:
    data_files.append(('share/' + package_name + '/launch', [f]))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={},
)