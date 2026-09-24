import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'g1_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'g1_simulation', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf', 'unitree_g1'), glob('urdf/unitree_g1/*')),
        (os.path.join('share', package_name, 'config', 'unitree_g1'), glob('config/unitree_g1/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pin_pelvis = g1_simulation.pin_pelvis:main',
        ],
    },
)
