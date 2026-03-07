from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elite_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
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
            'pose_to_joint_controller = elite_arm_controller.movecontrol:main',
            'first_scan = elite_arm_controller.first_scan:main',
            'install_node = elite_arm_controller.install:main',
            'uninstall_node = elite_arm_controller.uninstall:main'
        ],
    },
)
