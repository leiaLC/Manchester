from setuptools import find_packages, setup

package_name = 'path_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'my_custom_msgs', 'rclpy'],
    zip_safe=True,
    maintainer='zuriel',
    maintainer_email='zuriel.tovar.m@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polygon_publisher = path_follower.polygon_publisher:main',
            'polygon_imu = path_follower.Polygon_IMU:main',
            'path_pub = path_follower.path_publisher:main',
            'controller = path_follower.controller:main',
            "localisation = path_follower.localisation:main",
            'closed_loop_controller = path_follower.closed_loop_controller:main'
        ],
    },
)
