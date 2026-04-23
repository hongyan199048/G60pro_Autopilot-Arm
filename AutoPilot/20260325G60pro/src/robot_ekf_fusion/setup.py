from setuptools import setup

package_name = 'robot_ekf_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G60Pro Team',
    maintainer_email='admin@example.com',
    description='G60Pro EKF 定位融合（备用方案，当前未启用）',
    license='MIT',
    entry_points={},
)