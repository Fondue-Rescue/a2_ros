from setuptools import find_packages, setup

package_name = 'a2_velocity_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vikash Jeyathevan',
    maintainer_email='vikash.aadesh@gmail.com',
    description='Publishes timed velocity commands via twist_mux',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_control = a2_velocity_control.velocity_control:main',
        ],
    },
)
