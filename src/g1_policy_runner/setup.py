from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'g1_policy_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/g1_policy_runner']),
    ('share/g1_policy_runner', ['package.xml']),
    (os.path.join('lib', 'g1_policy_runner', 'policy'), glob('g1_policy_runner/policy/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='lucasmeneghetti02@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'policy_runner = g1_policy_runner.policy_runner:main',
        ],
    },
)
