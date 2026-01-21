from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ai_planner_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rric',
    maintainer_email='rric@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'markers_pub = ai_planner_ros.markers_pub:main', 
        'moveit_pub = ai_planner_ros.moveit_pub:main',
        'scene_rviz_pub = ai_planner_ros.scene_rviz_pub:main',
        'scene_spawner = ai_planner_ros.scene_spawner:main',
        'calibrate_tray = ai_planner_ros.calibrate_tray:main',
        'scene_spawner_moveit = ai_planner_ros.scene_spawner_mov:main',
        'selected_grasp_pub = ai_planner_ros.selected_grasp_pub:main',
        ],
    },
)
 