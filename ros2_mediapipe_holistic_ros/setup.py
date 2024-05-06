from setuptools import find_packages, setup

package_name = 'ros2_mediapipe_holistic_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='dmartinelli1997@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe_holistic = ros2_mediapipe_holistic_ros.mediapipe_holistic:main',
        ],
    },
)
