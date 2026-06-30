from setuptools import setup

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mayuri',
    maintainer_email='mayuri@example.com',
    description='Autonomous Warehouse Robot - ROS2 Humble',
    license='MIT',
    entry_points={
        'console_scripts': [
            'astar_planner      = warehouse_robot.astar_planner:main',
            'lidar_avoider      = warehouse_robot.lidar_avoider:main',
            'object_detector    = warehouse_robot.object_detector:main',
            'mission_controller = warehouse_robot.mission_controller:main',
        ],
    },
)
