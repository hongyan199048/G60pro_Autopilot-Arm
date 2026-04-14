from setuptools import setup
from glob import glob
import os

package_name = 'robot_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/g60pro.urdf.xacro']),
        ('share/' + package_name + '/meshes/g60pro', glob('meshes/g60pro/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={},
)