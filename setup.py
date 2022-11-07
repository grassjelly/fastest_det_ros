import os
from glob import glob
from setuptools import setup

package_name = 'fastest_det_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        'fastest_det_ros.fastest_det',
        'fastest_det_ros.fastest_det.utils',
        'fastest_det_ros.fastest_det.module'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'fastest_det_ros/fastest_det/configs'), glob('fastest_det_ros/fastest_det/configs/*.names')),
        (os.path.join('share', package_name, 'fastest_det_ros/fastest_det/weights'), glob('fastest_det_ros/fastest_det/weights/*.pth')),
    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='Juan Miguel jimeno',
    maintainer_email='jimenojmm@gmail.com',
    description='Colconized fastest_det_ros',
    license='Apache 2.0',
)
