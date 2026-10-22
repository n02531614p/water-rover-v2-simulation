import os
from glob import glob
from setuptools import setup

package_name = 'waterdroid_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@nust.ac.zw',
    description='Control nodes for Waterdroid',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_navigator = waterdroid_control.mission_navigator:main',
        ],
    },
)
