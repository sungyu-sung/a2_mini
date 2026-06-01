from setuptools import find_packages, setup

package_name = 'turtlebot4_capture'

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
    maintainer='choijinwoo',
    maintainer_email='herofactorycjw1998@gmail.com',
    description='day2 웹캠/Oak-D 이미지 캡처 스크립트 (YOLO 학습 데이터 수집용)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capture_wc_image = turtlebot4_capture.2_1_a_capture_wc_image:main',
            'cont_capture_wc_image = turtlebot4_capture.2_1_b_cont_capture_wc_image:main',
            'capture_image = turtlebot4_capture.2_1_d_capture_image:main',
            'cont_capture_image = turtlebot4_capture.2_1_e_cont_capture_image:main',
        ],
    },
)
