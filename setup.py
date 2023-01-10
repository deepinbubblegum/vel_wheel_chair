import os
from glob import glob
from setuptools import setup

package_name = 'vel_wheel_chair'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaiwit',
    maintainer_email='chaiwit.p@groupmaker.co.th',
    description='vel wheel chair',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vel_wheel_chair = vel_wheel_chair.vel_wheel_chair_node:main"
        ],
    },
)
