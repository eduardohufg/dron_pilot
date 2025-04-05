from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dron_pilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardohufg',
    maintainer_email='eduardochavezmartin10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_aruco = detect_aruco.detect_aruco:main',
            'depth_camera = detect_aruco.depth_camera:main',
            'cmd_dron = detect_aruco.cmd_dron:main',
            'node_controller = detect_aruco.node_controller:main',
        ],
    },
)
