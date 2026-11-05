from setuptools import find_packages, setup

package_name = 'waterdroid_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@nust.ac.zw',
    description='Waterdroid mission state machine, simulated treatment train, arm and status/MQTT bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_navigator = waterdroid_control.mission_navigator:main',
            'mission_navigator_gt = waterdroid_control.mission_navigator_gt:main',
            'water_processor = waterdroid_control.water_processor:main',
            'arm_controller = waterdroid_control.arm_controller:main',
            'status_monitor = waterdroid_control.status_monitor:main',
        ],
    },
)
