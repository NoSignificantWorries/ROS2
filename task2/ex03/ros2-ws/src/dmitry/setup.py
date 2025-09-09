from setuptools import find_packages, setup

package_name = 'dmitry'

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
    maintainer='Dmitry Karpachev',
    maintainer_email='d.karpachev@g.nsu.ru',
    description='ex03 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = dmitry.my_node:main'
        ],
    },
)
