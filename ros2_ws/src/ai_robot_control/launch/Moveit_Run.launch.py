import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.245')
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Config Builder
    moveit_config_obj = ( 
        MoveItConfigsBuilder(
            context=context,
            robot_ip=robot_ip,
            dof=dof,
            robot_type=robot_type,
            add_gripper=add_gripper,
            ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .joint_limits() # Wichtig für Pilz Planner!
        .to_moveit_configs()
    )

    # 2. move robot node
    my_grasp_node = Node(
        package='ai_robot_control',      
        executable='moveit_run', 
        output='screen',
        emulate_tty=True,
        parameters=[
            moveit_config_obj.robot_description,
            moveit_config_obj.robot_description_semantic,
            moveit_config_obj.robot_description_kinematics,
            moveit_config_obj.planning_pipelines,
            moveit_config_obj.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [my_grasp_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
