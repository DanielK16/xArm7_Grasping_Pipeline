#!/bin/bash

# Activate conda environment
source ~/miniforge3/bin/activate

# Function to cleanup files
cleanup_files() {
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/annotated.png
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/color.png
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/depth.png
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/data.json
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/grasp_points.json
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/predicted_grasps.npy
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/selected_grasp.json
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/vertical_grasp_vis.png
    rm -f /home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/masks/*
}
cleanup_files

# Function to end the programm
cleanup_background() {
    echo "Cleaning up"
    kill $LIVE_PID 2>/dev/null
    pkill -f "RVIZ2"
    pkill -f "Visualization"
    pkill -f "visualization.launch.py"
    pkill -f "xarm7_planner_realmove.launch.py"

    wmctrl -c "RViz" 2>/dev/null
    wmctrl -c "Grasping Planner" 2>/dev/null

    pkill -9 -f "main_deterministic.py" 2>/dev/null
    killall -9 python3 2>/dev/null
    killall -9 rviz2 2>/dev/null
    exit 0
}
trap cleanup_background SIGINT

# Position the Windows correctly
position_window() {
    local TITLE=$1
    local GEOM=$2
    while ! wmctrl -l | grep -iq "$TITLE"; do
        sleep 0.5
    done
    wmctrl -r "$TITLE" -b remove,maximized_vert,maximized_horz
    wmctrl -r "$TITLE" -e "$GEOM"
    wmctrl -a "$TITLE"
}

# Launch Visualization (realsense, markes, scene, pointcloud_scene)
gnome-terminal --tab --title Visualization -- bash -c "
    source /opt/ros/humble/setup.bash
    source ~/xArm7_Grasping_Pipeline/ros2_ws/install/setup.bash
    ros2 launch ai_planner_ros visualization.launch.py
    " &

# Launch Rviz and Planner for xarm7 (moveit,rviz)
gnome-terminal --tab --title="RVIZ2" -- bash -c "
    source /opt/ros/humble/setup.bash
    source ~/xArm7_Grasping_Pipeline/ros2_ws/install/setup.bash
    ros2 launch xarm_planner xarm7_planner_realmove.launch.py add_gripper:=true add_realsense_d435i:=true robot_ip:=192.168.1.245
    " &

#Start Background Process for AI_Planner
echo "Starting Modules for AI_Planner"
conda activate AI_Planner
python3 ~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/main_deterministic.py & LIVE_PID=$!

# Main Loop
sleep 25

# Position Window for a good look
position_window "RViz" "0,0,0,568,779"
position_window "Grasping Planner" "0,640,0,1280,720"
position_window "rric@ubuntu:" "0,0,788,1038,412"
firefox --new-window "http://192.168.1.245:18333" &
position_window "UFactory Studio" "0,1000,777,920,423"

#################################################################################
####            MAIN
#################################################################################

# 1 Calibrate Tray
# echo "[Calibration]: Centering Tray"
# source /opt/ros/humble/setup.bash
# source ~/xArm7_Grasping_Pipeline/ros2_ws/install/setup.bash
# ros2 run ai_planner_ros calibrate_tray

# 2 Start Cleaning
#read -p ">>> Press Enter to start <<<"
while true; do
    #read -p ">>> Press Enter to start <<<"
    cleanup_files
    # main.py
    kill -SIGUSR1 $LIVE_PID
    echo "AI Planner triggered"
    while [ ! -f ~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/grasp_points.json ]; do
        sleep 0.2
    done

    # Start Moveit
    echo "Moving to target"
    source /opt/ros/humble/setup.bash
    source ~/xArm7_Grasping_Pipeline/ros2_ws/install/setup.bash
    ros2 launch ai_robot_control Moveit_Run.launch.py
    echo "Press Enter to Start Grasping Process"

done
