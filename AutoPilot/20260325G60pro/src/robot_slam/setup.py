from setuptools import setup

package_name = 'robot_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/config', ['config/cartographer_sim.lua', 'config/cartographer_real.lua']),
        ('share/' + package_name + '/scripts', ['robot_slam/save_map.py']),
    ],
    install_requires=['setuptools', 'pillow', 'pyyaml'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'save_map = robot_slam.save_map:main',
        ],
    },
)