import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final_challenge'

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
        'camera_publisher = final_challenge.camera_publisher:main',
        'line_detector = final_challenge.line_detector:main',
        'intersection_detector = final_challenge.intersection_detector:main',
        'line_controller = final_challenge.line_controller:main',
        'line_controller_signals = final_challenge.line_controller_signals:main',
        'traffic_light_node = final_challenge.traffic_light_node:main',
        'localisation_node = final_challenge.localisation_node:main',
        'standalone_maneuver_tester = final_challenge.standalone_maneuver_tester:main',
    	],
    },
)
