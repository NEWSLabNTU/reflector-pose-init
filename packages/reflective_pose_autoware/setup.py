import os
from glob import glob

from setuptools import setup

package_name = 'reflective_pose_autoware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Golf Cart Team',
    maintainer_email='dev@golfcart.org',
    description='Autoware handoff for the reflective board detector',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # The name is deliberately unchanged from the old single package:
            # it is what an Autoware launch file names.
            'board_pose_initializer = reflective_pose_autoware.initializer_node:main',
        ],
    },
)
