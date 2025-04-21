from setuptools import find_packages, setup

package_name = 'puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leialc',
    maintainer_email='ariadna.lau.100@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trayectory_2m = puzzlebot.trayectory_2m:main',
            'trayectory_try = puzzlebot.trayectory_try:main',
            'polygon_publisher = puzzlebot.polygon_publisher:main',
            'path_publisher = puzzlebot.path_publisher:main',
            'path_subscriber = puzzlebot.path_subscriber:main',
            'polygon_IMU = puzzlebot.polygon_IMU:main',
            'controller = puzzlebot.controller:main',
        ],
    },
)
