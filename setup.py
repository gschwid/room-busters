import os
from glob import glob
from setuptools import setup

package_name = 'room-busters'
python_package_name = 'room_busters'

setup(
    name=package_name,
    version='0.0.0',
    packages=[python_package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='gschwid@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dumb_vacuum = room_busters.dumb_vacuum:main',
            # 'coverage_vacuum = room_busters.coverage_vacuum:main',
            'dumb_vacuum_v2 = room_busters.dumb_vacuum_v2:main'
        ],
    },
)
