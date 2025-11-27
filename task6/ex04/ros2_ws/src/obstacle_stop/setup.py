from setuptools import find_packages, setup

package_name = 'obstacle_stop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/move_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bval4',
    maintainer_email='b_val4@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_stop_node = obstacle_stop.obstacle_stop_node:main',
        ],
    },
)
