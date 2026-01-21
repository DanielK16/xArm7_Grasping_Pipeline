import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'rgb_camera.color_profile': '1280x720x15',
            'depth_module.depth_profile': '640x480x15',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items()
    )
   
    xarm_moveit_pkg = get_package_share_directory('xarm_moveit_config')
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_moveit_pkg, 'launch', 'xarm7_moveit_realmove.launch.py')
        ),
        launch_arguments={
            'robot_ip': '192.168.1.245',
            'add_realsense_d435i': 'true', 
            'add_gripper': 'true'
        }.items()
    )
    delayed_xarm_launch = TimerAction(
            period=3.0,
            actions=[xarm_moveit_launch]
        )

    calibrate_tray_node = Node(
        package='ai_planner_ros',
        executable='calibrate_tray',
        name='calibrate_tray',
        output='screen',
        on_exit=Shutdown(),
        emulate_tty=True, 
        parameters=[{
            'use_sim_time': False
        }]
    )
    delayed_calibrate_node = TimerAction(
            period=10.0,
            actions=[calibrate_tray_node]
        )

    return LaunchDescription([
        realsense_launch,
        delayed_xarm_launch,
        delayed_calibrate_node
    ])