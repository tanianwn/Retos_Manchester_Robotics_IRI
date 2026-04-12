from setuptools import find_packages, setup

package_name = 'challenge2'

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
            'path_generator = challenge2.path_generator:main',
            'controlador = challenge2.controlador:main',
            'open_loop_ctrl = challenge2.open_loop_ctrl:main',
            
        ],
    },
)
