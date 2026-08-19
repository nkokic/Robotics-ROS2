from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_lv3'

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
    maintainer='nikola',
    maintainer_email='nikola@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrolling_point_gatherer = amr_lv3.patrolling_point_gatherer:main',
            'patrolling_point_navigator = amr_lv3.patrolling_point_navigator:main',
            'object_detector = amr_lv3.object_detector:main',
        ],
    },
)
