#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Attach to Robot arm mesh
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>

// Standard Libs
#include <future>
#include <vector>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include <sstream>

// Custom/Third Party
#include "mgi.h"      
#include "pilz.hpp"
#include "json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

// =================================================================================================
// STRUCTURE FOR GRASP POSE
// =================================================================================================
struct GraspTarget {
    geometry_msgs::msg::Pose grasp_pose;     
    geometry_msgs::msg::Pose pre_grasp_pose; 
};

// =================================================================================================
// 2. CONFIGURATION CLASS FOR JSON
// =================================================================================================
/* 
Loads Config Data from: /home/rric/ros2_ws/src/ai_robot_control/config/config_grasps.json
*/

class JsonConfig {
public:
    json data;
    bool loaded = false;
    const double deg2rad = M_PI/180.0;

    JsonConfig(const std::string& path) {
        std::ifstream f(path);
        if (f.is_open()) {
            try {
                data = json::parse(f);
                loaded = true;
            } catch (const std::exception& e) {
                printf("[Config] JSON Error: %s\n", e.what());
            }
        } else {
            printf("[Config] Could not open config file: %s\n", path.c_str());
        }
    } 
    
    // --- Getters ---
    std::vector<double> getDropJoints(const std::string& object_name) {
        std::vector<double> joints_rad;
        std::string key = data["drop_points"].contains(object_name) ? object_name : "no_label";
        if (data["drop_points"].contains(key)) {
            for (double val_deg : data["drop_points"][key]) joints_rad.push_back(val_deg * deg2rad); 
        }
        return joints_rad;
    }

    double getGripperPos(const std::string& object_name) {
         std::string key = data["gripper_pos"].contains(object_name) ? object_name : "no_label";
         return 0.85 - (double)data["gripper_pos"][key];
    }

    double getTcpOffset(const std::string& object_name) {
        std::string key = data["offset_tcp"].contains(object_name) ? object_name : "no_label";
        return data["offset_tcp"][key];
    }

    double getPreGraspOffset(const std::string& object_name) {
        std::string key = data["pre_grasp_offset"].contains(object_name) ? object_name : "no_label";
        return data["pre_grasp_offset"][key];
    }

    std::vector<double> getStartingJoints() {
        std::vector<double> joints_rad;
        if (data.contains("starting_pos")) { 
            for (double val_deg : data["starting_pos"]) joints_rad.push_back(val_deg * deg2rad); 
        }
        return joints_rad;
    }  

    std::vector<double> getCleaningPos() {
        std::vector<double> cartesian_pos;
        if (data.contains("cleaning_coords")) {
        for (double val : data["cleaning_coords"]) {
            cartesian_pos.push_back(val);
        }
        }
        return cartesian_pos;
    }

    std::vector<double> getDropPointsCartesian(const std::string& object_name) {
        std::vector<double> cartesian_pos;
        if (!data.contains("drop_points_cartesian")) return {};
        std::string key = data["drop_points_cartesian"].contains(object_name) ? object_name : "no_label";
    if (data["drop_points_cartesian"].contains(key)) {
        for (double val : data["drop_points_cartesian"][key]) {
            cartesian_pos.push_back(val);
        }
    }
    return cartesian_pos;
    }

};


// =================================================================================================
// 3. GRASP CLIENT (TF Logic)
// =================================================================================================
class GraspClient {
public:
    GraspClient(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(node_->get_logger(), "GraspClient initialized.");
    }

    // --- Processing Logic ---
    std::vector<geometry_msgs::msg::Pose> get_grasps_transformed(const std::string& topic_name, const std::string& target_frame) {
        std::vector<geometry_msgs::msg::Pose> result_poses;
        
        auto received_msg = wait_for_message(topic_name);
        if (!received_msg) return result_poses;

        std::string source_frame = received_msg->header.frame_id;
        
        // Wait for Transform
        try {
            auto transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, 5s);
            
            for (const auto& camera_pose : received_msg->poses) {
                geometry_msgs::msg::Pose robot_pose;
                tf2::doTransform(camera_pose, robot_pose, transform);
                result_poses.push_back(robot_pose);
            }
            RCLCPP_INFO(node_->get_logger(), "Transformed %zu grasps to %s.", result_poses.size(), target_frame.c_str());

        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF Error: %s", ex.what());
        }
        return result_poses;
    }

    std::vector<GraspTarget> process_grasps(
        const std::vector<geometry_msgs::msg::Pose>& input_grasps,
        const std::string& detected_label,
        double tcp_offset,       
        double pre_grasp_offset
    ) {
        std::vector<GraspTarget> processed_targets;
        // Constants for strategies
        const std::vector<std::string> STRATEGY_GRASPNET = {"bowl_orange", "bowl_green", "cup"};
        const std::vector<std::string> STRATEGY_VERTICAL = {"spoon_orange", "chopstick"};

        for (const auto& raw_pose : input_grasps) {
            GraspTarget target;
            
            tf2::Vector3 pos(raw_pose.position.x, raw_pose.position.y, raw_pose.position.z);
            tf2::Quaternion quat;
            tf2::fromMsg(raw_pose.orientation, quat);

            // 1. TCP Offset (Back off along Z)
            tf2::Matrix3x3 m_orig(quat);
            tf2::Vector3 approach_vector = -m_orig.getColumn(2); 
            tf2::Vector3 pos_with_tcp = pos - (approach_vector * tcp_offset);

            // 2. Rotation Correction
            tf2::Quaternion q_correction(0,0,0,1);
            if (std::find(STRATEGY_GRASPNET.begin(), STRATEGY_GRASPNET.end(), detected_label) != STRATEGY_GRASPNET.end()) {
                q_correction.setRPY(0.0, 0.0, -M_PI/2.0); 
            } else if (std::find(STRATEGY_VERTICAL.begin(), STRATEGY_VERTICAL.end(), detected_label) != STRATEGY_VERTICAL.end()) {
                q_correction.setRPY(M_PI, 0, 0); 
            }
            
            tf2::Quaternion q_final = quat * q_correction; 
            q_final.normalize();

            // 3. Pre-Grasp Calculation
            tf2::Matrix3x3 m_corr(q_final);
            tf2::Vector3 approach_corr = -m_corr.getColumn(2);
            tf2::Vector3 pos_pre = pos_with_tcp + (approach_corr * pre_grasp_offset);
            
            // Fill Target
            target.grasp_pose.position.x = pos_with_tcp.x();
            target.grasp_pose.position.y = pos_with_tcp.y();
            target.grasp_pose.position.z = pos_with_tcp.z();
            target.grasp_pose.orientation = tf2::toMsg(q_final);

            target.pre_grasp_pose.position.x = pos_pre.x();
            target.pre_grasp_pose.position.y = pos_pre.y();
            target.pre_grasp_pose.position.z = pos_pre.z();
            target.pre_grasp_pose.orientation = tf2::toMsg(q_final);

            processed_targets.push_back(target);
        }
        return processed_targets;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::PoseArray::SharedPtr wait_for_message(const std::string& topic) {
        auto promise = std::make_shared<std::promise<geometry_msgs::msg::PoseArray::SharedPtr>>();
        auto future = promise->get_future();
        
        rclcpp::QoS qos(1);
        qos.transient_local().reliable();

        auto sub = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            topic, qos, [promise](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                try { promise->set_value(msg); } catch(...) {}
            });

        RCLCPP_INFO(node_->get_logger(), "Waiting for grasps on '%s'...", topic.c_str());
        if (future.wait_for(5s) == std::future_status::ready) {
            return future.get();
        }
        return nullptr;
    }
};

// =================================================================================================
// 4. HELPER FUNCTIONS
// =================================================================================================

// Safely loads the detected label from YOLO output
std::string load_detected_label(const std::string& path, rclcpp::Logger logger) {
    std::ifstream f(path);
    if (!f.is_open()) {
        RCLCPP_ERROR(logger, "Could not open YOLO file: %s", path.c_str());
        return "no_label";
    }
    try {
        json j = json::parse(f);
        if (j.is_array() && !j.empty() && j[0].contains("label")) {
            std::string lbl = j[0]["label"];
            RCLCPP_INFO(logger, "YOLO Detection: '%s'", lbl.c_str());
            return lbl;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "YOLO JSON Error: %s", e.what());
    }
    return "no_label";
}

// Ensures the robot controller is active and sending joint states
bool wait_for_robot_connection(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group) {
    auto start = node->get_clock()->now();
    RCLCPP_INFO(node->get_logger(), "Connecting to Robot...");
    while (rclcpp::ok()) {
        if (move_group.getCurrentState(2.0)) return true;
        
        if ((node->get_clock()->now() - start).seconds() > 10.0) {
            RCLCPP_ERROR(node->get_logger(), "TIMEOUT: Robot driver not responding!");
            return false;
        }
        RCLCPP_WARN(node->get_logger(), "Waiting for robot state...");
    }
    return false;
}

// =================================================================================================
// 5. MAIN
// =================================================================================================
// Set Planning Group
static const std::string PLANNING_GROUP = "xarm7";
static const std::string GRIPPER_GROUP = "xarm_gripper";

int main(int argc, char * argv[]) {
    
    // Initialise ROS and create the Node
    rclcpp::init(argc, argv);
    // allows to get parameters without declaring them first 
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("xarm_grasp_planner", options);
    
    // Allow laggy joint states 
    node->declare_parameter<double>("joint_state_monitor.max_expected_joint_state_delay", 0.5);

    // Get Parameters
    std::string config_path = node->declare_parameter<std::string>("config_path", "/home/rric/xArm7_Grasping_Pipeline/ros2_ws/src/ai_robot_control/config/config_grasps.json");
    std::string yolo_path = node->declare_parameter<std::string>("yolo_path", "/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/grasp_points.json");
    std::string grasp_topic = node->declare_parameter<std::string>("grasp_topic", "/moveit_grasp_candidates");

    // Executor for TF/MoveIt background tasks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Setup MoveIt
    using moveit::planning_interface::MoveGroupInterface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Initialise move_group and gripper_group
    MoveGroupInterface move_group(node, PLANNING_GROUP);
    MoveGroupInterface gripper_group(node, GRIPPER_GROUP);
    
    if (!wait_for_robot_connection(node, move_group)) return -1;

    // Initiliaze Classes 
    Pilz pilz_robot(node, move_group);
    GraspClient grasp_client(node);
    
    // Load Data
    std::string detected_label = load_detected_label(yolo_path, node->get_logger());
    JsonConfig config(config_path);
    if (!config.loaded) return -1;

    // Retrieve specific parameters for the object
    auto drop_joints = config.getDropJoints(detected_label);
    auto start_joints = config.getStartingJoints();
    double pre_grasp_offset = config.getPreGraspOffset(detected_label);
    double gripper_pos = config.getGripperPos(detected_label);
    double tcp_offset = config.getTcpOffset(detected_label);
    auto cleaning_coords = config.getCleaningPos();
    auto drop_points_cartesian = config.getDropPointsCartesian(detected_label);

    // Grasp Computation
    auto grasps = grasp_client.get_grasps_transformed(grasp_topic, "link_base");
    if (grasps.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No grasp candidates received. Aborting.");
        return 0;
    }
    auto targets = grasp_client.process_grasps(grasps, detected_label, tcp_offset, pre_grasp_offset);

    // Get Object Pose of detected_label in Planning Scene
    geometry_msgs::msg::Pose final_pose;
    final_pose.orientation.w = 1.0; 
    bool pose_received = false;
    double z_offset = 0.05; //because of mesh center

    // we add a mesh for the bowl so the gripper is aware of the object
    if (detected_label == "bowl" || detected_label == "bowl_orange" || detected_label == "bowl_green") {
        auto grasp_vector = grasp_client.get_grasps_transformed("/selected_grasp_pose", "link_base");
        if (grasp_vector.empty()) {
            RCLCPP_ERROR(node->get_logger(), "MESH-LOGIC: No pose received on /selected_grasp_pose! Frame-ID check needed.");
        } else {
            RCLCPP_INFO(node->get_logger(), "MESH-LOGIC: Pose received! X: %.2f", grasp_vector[0].position.x);
            final_pose = grasp_vector[0]; 
            final_pose.position.z += z_offset;
            pose_received = true;
        }
    }

    // =============================================================================================
    // EXECUTION STATE MACHINE
    // =============================================================================================

    // 1. Move Home
    RCLCPP_INFO(node->get_logger(), ">>> PHASE 1: Moving Home");
    pilz_robot.PTP(start_joints, false, 0.5, 0.5);

    // 2. Set Gripper to Starting Position
    RCLCPP_INFO(node->get_logger(), ">>> Adjusting Starting Gripper Position to %f", gripper_pos);
    gripper_group.setJointValueTarget("drive_joint", gripper_pos);
    gripper_group.move();
    
    // Possible Setup MoveIt for Grasping 
            // Selection of Planner
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");         //PILZ
    //move_group.setPlanningPipelineId("ompl");                                 //OMPL
            // How many times the solver tries to find a valid path
    move_group.setNumPlanningAttempts(3);
            // Max Speed Velocity
    move_group.setMaxVelocityScalingFactor(0.4);
            // Max Acceleration 
    move_group.setMaxAccelerationScalingFactor(0.3);
            // How much each joint can deviate in radius 
    move_group.setGoalJointTolerance(0.02);              
            // Allowed Rotation error in radians
    move_group.setGoalOrientationTolerance(0.02);
            // Allowed Distance Error in mm
    move_group.setGoalPositionTolerance(0.002);
            // Maximum Time allowed for planning
    move_group.setPlanningTime(3.0);

    // 3. Attempt Grasp Loop (Pre_Grasp + Grasp)
    bool grasp_success = false;

    // Try until a grasp target is successfull
    for (size_t i = 0; i < targets.size(); i++) {
        RCLCPP_INFO(node->get_logger(), ">>> PHASE 2: Attempting Grasp Candidate %zu", i);
        
        // A. Approach Pre-Grasp (PTP)
        move_group.setPlanningPipelineId("pilz_industrial_motion_planner"); 
        move_group.setPlannerId("PTP");
        move_group.setPoseTarget(targets[i].pre_grasp_pose);
        
        if (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
            
            // B. Final Approach (PTP)
            move_group.setPlanningPipelineId("pilz_industrial_motion_planner"); 
            move_group.setPlannerId("PTP"); 
            move_group.setPoseTarget(targets[i].grasp_pose);
            
            if (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
                grasp_success = true;
                RCLCPP_INFO(node->get_logger(), "Grasp Candidate %zu reached successfully!", i);
                break; 
            }
            else {
            RCLCPP_ERROR(node->get_logger(), "Final Grasp %zu failed! Backing off to Pre-Grasp...", i);
            move_group.setPoseTarget(targets[i].pre_grasp_pose);
            move_group.move();
            }
        }
    }

    if (!grasp_success) {
        RCLCPP_ERROR(node->get_logger(), "All grasp attempts failed. Returning Home.");
        pilz_robot.PTP(start_joints, false, 0.3, 0.3);
        rclcpp::shutdown(); //shutdown if failed!
        return 0;
    }

    // 4. Close Gripper
    RCLCPP_INFO(node->get_logger(), ">>> PHASE 3: Grasping, Closing Gripper");
    gripper_group.setNamedTarget("close");  //setNamedTarget is defined in the SRDF 
    gripper_group.move();

    //4.5 Add Mesh as CollisionObject to Planning Scene 
    //If detected label is bowl add a mesh of the bowl!
    bool object_added = false;
    if (pose_received) {
        RCLCPP_INFO(node->get_logger(), "Adding Mesh: %s", detected_label.c_str());
        Eigen::Vector3d scale(1.0, 1.0, 1.0); 
        // Create Mesh
        std::string mesh_path = "file:///home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/PC_2_MESH/output_data/" + detected_label + ".stl";
        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path, scale);

        if (m) {
            // Create Collision Object
            moveit_msgs::msg::CollisionObject bowl_obj;
            bowl_obj.header.frame_id = "link_base"; 
            bowl_obj.id = detected_label; 
            
            // Convert Mesh
            shape_msgs::msg::Mesh mesh_msg;
            shapes::ShapeMsg mesh_shape_msg;
            shapes::constructMsgFromShape(m, mesh_shape_msg);
            mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_shape_msg);

            bowl_obj.meshes.push_back(mesh_msg);
            bowl_obj.mesh_poses.push_back(final_pose); 
            bowl_obj.operation = bowl_obj.ADD;

            planning_scene_interface.applyCollisionObject(bowl_obj);
            RCLCPP_INFO(node->get_logger(), "Object '%s' added to scene.", bowl_obj.id.c_str());
            delete m;
            object_added = true;
        }
    }
    // adding a basic collision object (cube) to prevent collison
    else {
        moveit_msgs::msg::CollisionObject box_obj;
        box_obj.header.frame_id = "link_base";
        box_obj.id = detected_label; 

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.06; // 5cm breit
        primitive.dimensions[primitive.BOX_Y] = 0.06; // 5cm tief
        primitive.dimensions[primitive.BOX_Z] = 0.07; // 5cm hoch

        box_obj.primitives.push_back(primitive);

        auto collison_object_pose = move_group.getCurrentPose().pose;
        box_obj.primitive_poses.push_back(collison_object_pose);
        box_obj.operation = box_obj.ADD;

        planning_scene_interface.applyCollisionObject(box_obj);
        RCLCPP_INFO(node->get_logger(), "Basic collision object added for '%s' to scene.", detected_label.c_str());
        object_added = true;
    }

    // Attach Object
    if (object_added) {
        std::vector<std::string> touch_links = {
            "left_outer_knuckle", "left_finger", "left_inner_knuckle",
            "right_outer_knuckle", "right_finger", "right_inner_knuckle"
        };
        // Attach the object using the ID we used during ADD (the label)
        move_group.attachObject(detected_label, move_group.getEndEffectorLink(), touch_links);
        RCLCPP_INFO(node->get_logger(), "Object '%s' attached to MoveIt.", detected_label.c_str());
    }

    // 5. Lift Object (LIN)
    RCLCPP_INFO(node->get_logger(), ">>> PHASE 4: Lifting Object straight up");
    move_group.setPlannerId("LIN");
    auto lift_pose = move_group.getCurrentPose().pose;
    lift_pose.position.z += 0.15;
    move_group.setPoseTarget(lift_pose);
    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node->get_logger(), "LIN Lift failed. Falling back to Cartesian Path...");
        // Fallback Logic
        std::vector<geometry_msgs::msg::Pose> waypoints {lift_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if (fraction > 0.8) move_group.execute(trajectory);
    }

    // 6. Transit to Cleaning Spot to Cartesian Coordiantes without spilling
    RCLCPP_INFO(node->get_logger(), ">>> PHASE 5: Transit");
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");   
    move_group.setPlannerId("LIN");
    auto cleaning_position = move_group.getCurrentPose().pose;
    //Orientation Coordinates stay the same to avoid spilling: We move in Cartesian Coordiantes
    cleaning_position.position.x = cleaning_coords[0];
    cleaning_position.position.y = cleaning_coords[1];
    cleaning_position.position.z = cleaning_coords[2];
    RCLCPP_INFO(node->get_logger(), "Moving to Cleaning Position with LIN: [%.3f, %.3f, %.3f]", cleaning_position.position.x, cleaning_position.position.y, cleaning_position.position.z);
    move_group.setPoseTarget(cleaning_position);
    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {        
        // Fallback with PTP if LIN fails
        RCLCPP_INFO(node->get_logger(), "Moving to Cleaning Position with PTP as Fallbaack:");
        move_group.setPlannerId("PTP"); 
        move_group.setPoseTarget(cleaning_position);
        move_group.move();
        }

    // 6. Transit to Cleaning/Safe Spot (Intermediate PTP)
    // RCLCPP_INFO(node->get_logger(), ">>> PHASE 5: Transit");
    // move_group.setPlannerId("PTP");
    // auto transit_pose = move_group.getCurrentPose().pose;
    // transit_pose.position.y -= 0.3; // Move sideways
    // move_group.setPoseTarget(transit_pose);
    // move_group.move();

    // RCLCPP_INFO(node->get_logger(), ">>> PHASE 5.5: Transit to safe drop spot");
    // move_group.setPlannerId("PTP");
    // auto up_pose = move_group.getCurrentPose().pose;
    // up_pose.position.z += 0.2; // Move up
    // move_group.setPoseTarget(up_pose);
    // move_group.move();

    // 7. Move To Drop Point
    if (detected_label == "cup" || detected_label == "bowl_orange" || detected_label == "bowl" || detected_label == "bowl_green"){
        move_group.setPlanningPipelineId("pilz_industrial_motion_planner");   
        move_group.setPlannerId("PTP");
        auto drop_point_cartesian = move_group.getCurrentPose().pose;
        //Orientation Coordinates stay the same to avoid spilling: We move in Cartesian Coordiantes
        drop_point_cartesian.position.x = drop_points_cartesian[0];
        drop_point_cartesian.position.y = drop_points_cartesian[1];
        drop_point_cartesian.position.z = drop_points_cartesian[2];
        RCLCPP_INFO(node->get_logger(),  ">>> PHASE 6: Moving to Drop Point in Cartesian: [%.2f, %.2f, %.2f]",drop_point_cartesian.position.x, drop_point_cartesian.position.y, drop_point_cartesian.position.z);
        move_group.setPoseTarget(drop_point_cartesian);
        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {        
            // Fallback with Joint coordinates
            RCLCPP_INFO(node->get_logger(), "Moving to Drop Position with Joint Coordinates as Fallbaack:");
            pilz_robot.PTP(drop_joints, false, 0.4, 0.4);
        }
    }
    else {
        RCLCPP_INFO(node->get_logger(), 
        ">>> PHASE 6: Moving to Drop Point in Joints Pos: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        drop_joints[0], drop_joints[1], drop_joints[2], 
        drop_joints[3], drop_joints[4], drop_joints[5], drop_joints[6]);
        pilz_robot.PTP(drop_joints, false, 0.4, 0.4);
    }

    // Open Gripper
    gripper_group.setNamedTarget("open");
    gripper_group.move();

    // 7.5 Try to Detach and Delete Object from Planning Scene
    RCLCPP_INFO(node->get_logger(),"Detaching Object");
    move_group.detachObject(detected_label);
    RCLCPP_INFO(node->get_logger(), "Object '%s' detached from gripper.", detected_label.c_str());
    std::vector<std::string> object_ids = {detected_label};
    planning_scene_interface.removeCollisionObjects(object_ids);
    RCLCPP_INFO(node->get_logger(), "Object '%s' removed from planning scene.", detected_label.c_str());


    // 8. Return Home
    RCLCPP_INFO(node->get_logger(), ">>> PHASE 7: Done");
    pilz_robot.PTP(start_joints, false, 0.5, 0.5);

    RCLCPP_INFO(node->get_logger(), ">>> Grasping done. Shuting down...");
    rclcpp::shutdown();
    return 0;
}
