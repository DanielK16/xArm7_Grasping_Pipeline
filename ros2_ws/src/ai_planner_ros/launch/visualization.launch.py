import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    
    # Scene Spawner (Static objects like table)
    scene_spawner_node = Node(
        package='ai_planner_ros',  
        executable='scene_spawner_moveit', 
        name='moveit_py_scene_spawner',
        output='screen',
        parameters=[],
    )

    # Publish Pointcloud for Visualization
    scene_rviz_pub_node = Node(
        package='ai_planner_ros',  
        executable='scene_rviz_pub', 
        name='scene_rviz_pub',
        output='screen',
        parameters=[],
    )

    # Marker Publisher (Grasp visualization)
    markers_pub_node = Node(
        package='ai_planner_ros',  
        executable='markers_pub', 
        name='markers_pub',
        output='screen',
        parameters=[],
    )

    # Publish all grasps
    moveit_pub_node = Node(
        package='ai_planner_ros',  
        executable='moveit_pub', 
        name='moveit_pub',
        output='screen',
        parameters=[],
    )

    # Publish selected grasp pose for mesh attachment
    selected_grasp_node = Node(
        package='ai_planner_ros',  
        executable='selected_grasp_pub', 
        name='selected_grasp_pub',
        output='screen',
        parameters=[
        ],
    )


    # Launch Realsense ros camera
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

    return LaunchDescription([
        scene_spawner_node,
        scene_rviz_pub_node,
        markers_pub_node,
        moveit_pub_node,
        selected_grasp_node,
        realsense_launch
    ])