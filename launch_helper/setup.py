from setuptools import find_packages, setup

package_name = 'launch_helper'

setup(
    name=package_name,
    version='1.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Afzal Ahmad',
    maintainer_email='afzalhmd14@gmail.com',
    description='ROS2 launch-helper library',
    license='BSD 3-Clause',
    py_modules=['launch_helper.helper'],
    entry_points={
        'console_scripts': [
            'dummy_node = launch_helper.dummy_node:main',
        ],
    },
)
