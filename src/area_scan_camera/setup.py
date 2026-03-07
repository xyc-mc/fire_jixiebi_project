from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'area_scan_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    package_data={
        package_name: ['*.so'],
    },
    install_requires=['setuptools', 'numpy>=1.26.0', 'open3d>=0.19.0', 'python3-pcl'],
    zip_safe=True,
    maintainer='neepu-jk',
    maintainer_email='neepu-jk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
         'camera_control = area_scan_camera.camera_control:main',
         'program_entrance = area_scan_camera.program_entrance:main'
        ],
    },
)
