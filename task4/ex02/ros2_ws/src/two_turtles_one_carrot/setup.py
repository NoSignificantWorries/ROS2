import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'two_turtles_one_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dmitry.karpachev@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'carrot = two_turtles_one_carrot.carrot:main',
            'broadcaster = two_turtles_one_carrot.broadcaster:main',
            'listener = two_turtles_one_carrot.listener:main',
        ],
    },
)
