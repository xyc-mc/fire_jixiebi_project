from setuptools import setup
import os
from glob import glob

package_name = 'handeye_coord_transformer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='手眼坐标变换系统',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handeye_transform_node = handeye_coord_transformer.handeye_transformer_node:main',
            'plane_normal_to_rpy_node = handeye_coord_transformer.plane_normal_to_rpy_node:main',
        ],
    },
)