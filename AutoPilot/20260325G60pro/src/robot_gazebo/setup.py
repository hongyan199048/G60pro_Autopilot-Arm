from setuptools import setup
from glob import glob

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={},
)