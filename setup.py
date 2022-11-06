import os
from glob import glob
from setuptools import setup

package_name = 'yolo_fastest_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        'yolo_fastest_ros.yolo_fastest',
        'yolo_fastest_ros.yolo_fastest.data',
        'yolo_fastest_ros.yolo_fastest.model',
        'yolo_fastest_ros.yolo_fastest.model.backbone',
        'yolo_fastest_ros.yolo_fastest.modelzoo',
        'yolo_fastest_ros.yolo_fastest.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'yolo_fastest_ros/yolo_fastest/data'), glob('yolo_fastest_ros/yolo_fastest/data/*.names')),
        (os.path.join('share', package_name, 'yolo_fastest_ros/yolo_fastest/modelzoo'), glob('yolo_fastest_ros/yolo_fastest/modelzoo/*.pth'))

    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='Juan Miguel jimeno',
    maintainer_email='jimenojmm@gmail.com',
    description='Colconized yolo_fastest_ros',
    license='TBD',
)
