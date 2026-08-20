from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'amr_lv4'

# Collect model files with their subdirectories preserved
model_data = []
for root, dirs, files in os.walk('models'):
    if not files:
        continue  # skip empty dirs
    # e.g. root = "models/aruco_marker_23/materials/textures"
    src_files = [os.path.join(root, f) for f in files]
    # Install under share/amr_lv4/<root> (so structure is preserved)
    install_dir = os.path.join('share', package_name, root)
    model_data.append((install_dir, src_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ] + model_data,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='valentin.simundic@ferit.hr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_localization = amr_lv4.aruco_localization_node:main',
            'gather_points = amr_lv4.gather_points:main',
            'patrol_points = amr_lv4.patrol_points:main',
            'collect_marker_poses = amr_lv4.collect_marker_poses:main',
            'aruco_patrol = amr_lv4.aruco_patrol_node:main',
        ],

    },
)
