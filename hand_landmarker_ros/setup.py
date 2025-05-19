from setuptools import find_packages, setup

package_name = 'hand_landmarker_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=False,
    maintainer='Piatan Sfair Palar',
    maintainer_email='piatan@alunos.utfpr.edu.br',
    description='ROS2 node for hand landmark detection using MediaPipe',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_landmarker_node = hand_landmarker_ros.hand_landmarker_node:main',
        ],
    },
)
