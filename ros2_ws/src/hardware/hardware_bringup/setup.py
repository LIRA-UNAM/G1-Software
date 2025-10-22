from setuptools import find_packages, setup
import os

package_name = 'hardware_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/g1_init.launch.py']),
        (os.path.join('lib', package_name), ['scripts/g1_initializer']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Angel',
    maintainer_email='mglp@gmail.com',
    description='Bringup e inicialización del G1.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'g1_initializer = hardware_bringup.g1_initializer:main',
        ],
    },
)
