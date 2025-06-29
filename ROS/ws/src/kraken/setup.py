from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kraken'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'custom'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = kraken.planner:main',
            'controller = kraken.controller:main',
            'computer_vision = kraken.computer_vision:main',
            'state_estimator = kraken.state_estimator:main',
        ],
    },
)
