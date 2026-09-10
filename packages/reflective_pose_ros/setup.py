import os
from glob import glob

from setuptools import setup

package_name = 'reflective_pose_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        # The node's wiring parameters. The detector file itself is not
        # copied here: reflective_pose_core installs its own to share/, and
        # the launch file points there.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Golf Cart Team',
    maintainer_email='dev@golfcart.org',
    description='Board detection node, launch files and debug viewers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'board_detector_node = reflective_pose_ros.detector_node:main',
            'board_scene_publisher = reflective_pose_ros.scene_publisher:main',
            'anchor_debug_viewer = reflective_pose_ros.anchor_debug_viewer:main',
        ],
    },
)
