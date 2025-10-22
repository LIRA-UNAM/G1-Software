from setuptools import find_packages, setup

package_name = 'twist_to_g1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/g1_cmdvel.launch.py']),
        (f'share/{package_name}/config', ['config/bridge_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedoctor',
    maintainer_email='marco.negrete@ingenieria.unam.edu',
    description='Bridge /cmd_vel -> Unitree G1.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_g1 = twist_to_g1.twist_to_g1:main',
            'odom_to_tf = twist_to_g1.odom_to_tf:main',
        ],
    },
)
