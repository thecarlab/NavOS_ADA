from setuptools import find_packages, setup

package_name = 'util'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='yuxw',
    maintainer_email='yuxw@udel.edu',
    description='util ros2 node used to transfer sensor coordinate',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'imu_transformer_node = util.imu_transformer_node:main',
            'lidar_transformer_node = util.lidar_transformer_node:main',
        ],
    },
)
