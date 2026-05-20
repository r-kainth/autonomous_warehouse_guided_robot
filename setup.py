from setuptools import setup
from glob import glob
import os

package_name = 'forklift_bot'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    description='Forklift Robot with Autonomous Box Detection and Pickup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = forklift_bot.mission_controller:main',
            'box_detector = forklift_bot.box_detector:main',
            'forklift_controller = forklift_bot.forklift_controller:main',
        ],
    },
)