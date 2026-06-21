import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fonduerescue'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'), glob('configs/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RSS Team 6',
    maintainer_email='wagnermo@student.ethz.ch',
    description='ROS 2 package for RSS Team 6',
    license='GPL',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wp_exec_node = fonduerescue.wp_exec:main'
        ],
    },
)
