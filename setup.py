from setuptools import setup
from glob import glob

package_name = 'thymio_simulation'
controllers = f'{package_name}.controllers'
utils = f'{package_name}.utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, controllers, utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ## NOTE: you must add this line to use launch files
        # Instruct colcon to copy launch files during package build 
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=['Alind Xhyra', 'Federico Pallotti'],
    maintainer_email='alind.xhyra@usi.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_8_trajectory = thymio_simulation.node_8_trajectory:main',
            'node_points_2_3 = thymio_simulation.node_points_2_3:main'
        ],
    },
)
