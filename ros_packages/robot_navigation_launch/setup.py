import os
from glob import glob


from setuptools import find_packages, setup

package_name = 'robot_navigation_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('robot_navigation_launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('robot_navigation_launch', 'config/*.yaml')))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='animated',
    maintainer_email='chrisjnassif@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
