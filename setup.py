import os
from glob import glob

from setuptools import setup

package_name = 'golfcart_board_initializer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.simulation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Golf Cart Team',
    maintainer_email='dev@golfcart.org',
    description='Cold-start localization from a retroreflective board',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'board_pose_initializer = golfcart_board_initializer.node:main',
            'board_scene_publisher = golfcart_board_initializer.scene_publisher:main',
            'anchor_map_to_board = golfcart_board_initializer.anchor_cli:main',
        ],
    },
)
