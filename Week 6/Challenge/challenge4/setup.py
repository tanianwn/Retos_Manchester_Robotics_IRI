import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'challenge4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='puzzlebot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'line_follower = challenge4.line_follower:main',
        'line_follower2 = challenge4.line_follower2:main',
        'lane_follower_angle = challenge4.lane_follower_angle:main',
        'camera_publisher = challenge4.camera_publisher:main',
        'lane_follower_scanline = challenge4.lane_follower_scanline:main',
        'line_detector = challenge4.line_detector:main',
        'line_controller = challenge4.line_controller:main',
        'traffic_light_node = challenge4.traffic_light_node:main',
    	],
    },
)
