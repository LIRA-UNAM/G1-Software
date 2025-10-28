from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hardware_bringup'

def files_if_dir(path_glob):
    return glob(path_glob) if any(True for _ in glob(path_glob)) else []

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', files_if_dir('launch/*')),
        (f'share/{package_name}/urdf',   files_if_dir('urdf/*')),
        (f'share/{package_name}/meshes', files_if_dir('meshes/*')),
        (f'share/{package_name}/rviz',   files_if_dir('rviz/*')),
        (os.path.join('lib', package_name), files_if_dir('scripts/*')),
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

