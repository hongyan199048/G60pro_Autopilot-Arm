from setuptools import setup

package_name = 'arm_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_vision.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G60Pro Team',
    maintainer_email='admin@example.com',
    description='G60Pro 机械臂视觉引导：YOLO + ICP 位姿估计',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = ' + package_name + '.vision_node:main',
            'icp_node = ' + package_name + '.icp_node:main',
        ],
    },
)
