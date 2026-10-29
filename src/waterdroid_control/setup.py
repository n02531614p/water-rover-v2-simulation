from setuptools import find_packages, setup

package_name = 'waterdroid_control'

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
    maintainer='tyler',
    maintainer_email='tyler@todo.todo',
    description='Waterdroid autonomous control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_navigator = waterdroid_control.mission_navigator:main',
        ],
    },
)
