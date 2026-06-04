import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot4_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sungyu',
    maintainer_email='sunq0726@gmail.com',
    description='TurtleBot4 카메라 + YOLO(best.pt) bounding box 발행/구독 미니프로젝트 패키지',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = turtlebot4_yolo.yolo_detector:main',
            'yolo_viewer = turtlebot4_yolo.yolo_viewer:main',
        ],
    },
)
