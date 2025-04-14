from setuptools import find_packages, setup

package_name = 'vision_examples_utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Afzal Ahmad',
    maintainer_email='afzalhmd14@gmail.com',
    description='TODO: Package description',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_pub = vision_examples_utils.dummy_publisher:main',
        ],
    },
)
