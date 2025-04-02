from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pc2_to_grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='aries94leifu@gmail.com',
    description='pc2 to grid',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc2grid = pc2_to_grid.pc2grid:main',
            'pc2transform = pc2_to_grid.pc2_transform:main',
            'map_saver = pc2_to_grid.map_saver:main',
            'pc2map_saver = pc2_to_grid.pc2map:main'
        ],
    },
)
