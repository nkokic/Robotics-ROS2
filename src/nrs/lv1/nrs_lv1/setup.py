from setuptools import find_packages, setup

package_name = 'nrs_lv1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikola',
    maintainer_email='nikola1.kokic@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtleControl = nrs_lv1.TurtleControl:main',
            'znamenitostiNode = nrs_lv1.TurtlePub:main',
            'showHistory = nrs_lv1.ShowHistory:main',
        ],
    },
)
