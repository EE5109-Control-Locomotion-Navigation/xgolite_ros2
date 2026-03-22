from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltag_workspace'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian Deegan',
    maintainer_email='brian.deegan@universityofgalway.ie',
    description='AprilTag workspace boundary detection and visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workspace_manager = apriltag_workspace.workspace_node:main',
            'v4l2_opencv_cam = apriltag_workspace.v4l2_opencv_cam_node:main',
        ],
    },
)
