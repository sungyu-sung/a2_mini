import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_mission'

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
    description='CCTV YOLO 차량 감지 → TurtleBot4 출동·접근·정지 미션 패키지',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'usb_car_undock = car_mission.usb_car_undock:main',
            'topview_undock = car_mission.topview_undock:main',
            'undock_navigator = car_mission.undock_navigator:main',
            'yolo_depth_detector = car_mission.yolo_depth_detector:main',
            'car_tracking = car_mission.car_tracking:main',
            'mission_orchestrator = car_mission.mission_orchestrator:main',
            'dori_search_car = car_mission.dori_search_car:main',
        ],
    },
)
