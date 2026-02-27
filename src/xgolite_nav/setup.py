from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xgolite_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian Deegan',
    maintainer_email='brian.deegan@universityofgalway.ie',
    description='A* and pure pursuit autonomous navigation for XGO Lite',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_node = xgolite_nav.localization_node:main',
            'astar_planner_node = xgolite_nav.astar_planner_node:main',
            'pure_pursuit_node = xgolite_nav.pure_pursuit_node:main',
            'nav_orchestrator_node = xgolite_nav.nav_orchestrator_node:main',
        ],
    },
)
