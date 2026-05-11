import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'half_term_challenge'

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
    maintainer='inaki',
    maintainer_email='inaki@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localisation = half_term_challenge.localisation_node:main',
            'path_generator = half_term_challenge.path_generator:main',
            'controller = half_term_challenge.controller_node:main',
            'traffic_light_node = half_term_challenge.traffic_light_node:main',
            'fake_robot = half_term_challenge.fake_robot_node:main',
            'traffic_debugging = half_term_challenge.traffic_debugging:main',
            'opencv_act1 = half_term_challenge.opencv_act1:main',
            'controller_double = half_term_challenge.controller_double:main',
            'controller2 = half_term_challenge.controller2:main',
            'controller_v4 = half_term_challenge.controller_v4:main',
            'traffic_light_node2 = half_term_challenge.traffic_light_node2:main',
        ],
    },
)
