from setuptools import setup
import os
from glob import glob

package_name = 'bb_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 
              package_name + '/bev', 
              package_name + '/config', 
              package_name + '/vista',
              package_name + '/det3d'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alessandro Barbiero',
    maintainer_email='alessandro.barbiero@mail.polimi.it',
    description='Package to perform different detection algorithms',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = bb_detection.yolo_detector:main',
            'thermal_detector = bb_detection.thermal_detector:main',
            'lidar_detector = bb_detection.lidar_detector:main',
        ],
    },
)
