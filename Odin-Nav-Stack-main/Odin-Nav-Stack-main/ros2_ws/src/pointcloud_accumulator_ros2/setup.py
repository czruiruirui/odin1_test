from setuptools import setup, find_packages

package_name = 'pointcloud_accumulator_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/realtime_cloud_accumulator_ros2.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manifold Tech',
    maintainer_email='support@manifoldtech.com',
    description='ROS2 realtime point cloud accumulator for Odin1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'realtime_cloud_accumulator_ros2 = {package_name}.realtime_cloud_accumulator_ros2:main',
        ],
    },
)
