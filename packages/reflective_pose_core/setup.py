# colcon's ament_python build type invokes setup.py; pyproject.toml alone is
# invisible to it. This file and pyproject.toml describe the same distribution
# -- pip reads the latter, colcon this one -- so a dependency added to one must
# be added to the other.
import os
from glob import glob

from setuptools import setup

package_name = 'reflective_pose_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('reflective_pose_core/data/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Golf Cart Team',
    maintainer_email='dev@golfcart.org',
    description='ROS-free retroreflective board detector and map anchoring library',
    license='Apache-2.0',
    package_data={package_name: ['data/*.yaml']},
    include_package_data=True,
)
