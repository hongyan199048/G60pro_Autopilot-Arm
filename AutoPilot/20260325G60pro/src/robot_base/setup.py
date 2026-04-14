from setuptools import setup
import os

package_name = 'robot_base'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base.launch.py']),
        ('share/' + package_name + '/config', ['config/base.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G60Pro Team',
    maintainer_email='admin@example.com',
    description='G60Pro 4轮8驱底盘运动学解算',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_base_node = ' + package_name + '.robot_base_node:main',
        ],
    },
)