from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
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
    description='Package to perform vista detection algorithm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_detector = detector.lidar_detector:main',
        ],
    },
)
