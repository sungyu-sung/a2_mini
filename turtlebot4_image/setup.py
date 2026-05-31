from setuptools import find_packages, setup

package_name = 'turtlebot4_image'

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
    maintainer='rokey',
    maintainer_email='chanhwii20@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'image_publisher = turtlebot4_image.2_0_a_image_publisher:main',
        'image_subscriber = turtlebot4_image.2_0_b_image_subscriber:main',
        'data_publisher = turtlebot4_image.2_0_c_data_publisher:main',
        'data_subscriber = turtlebot4_image.2_0_d_data_subscriber:main',
        ],
    },
)
