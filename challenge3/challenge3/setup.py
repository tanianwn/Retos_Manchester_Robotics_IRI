from setuptools import find_packages, setup

package_name = 'challenge3'

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
    maintainer='marcos-allen',
    maintainer_email='marcos-allen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localisation_node = challenge3.localisation_node:main',
            'path_generator = challenge3.path_generator:main',
            'controller_node = challenge3.controller_node:main',
        ],
    },
)
