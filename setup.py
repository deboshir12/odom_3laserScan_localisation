from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch-файлы
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # urdf файлы
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # rviz конфиг
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        # gzBridge конфиг
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # gz world 
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deboshir',
    maintainer_email='deboshir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_to_rviz = robot_pkg.tf_to_rviz:main',
            'reading_laser_scan = robot_pkg.reading_laser_scan:main',
            'fixed_lidar = robot_pkg.fixed_lidar:main',
            'route_sender = robot_pkg.route_sender:main',
            'range_odom_corrector_map = robot_pkg.range_odom_corrector_map:main'
        ],
    },
)
