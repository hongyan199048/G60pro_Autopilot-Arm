from setuptools import setup
import os

package_name = 'robot_can'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/can.launch.py']),
    ],
    install_requires=['setuptools', 'cantools', 'python-can'],
    zip_safe=True,
    maintainer='G60Pro Team',
    maintainer_email='admin@example.com',
    description='G60Pro CAN 通信节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_node = ' + package_name + '.can_node:main',
        ],
    },
)