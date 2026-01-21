//**********************************************************************
// author : Edwin Ho
// Organization : Ngee Ann Polytechnic
// Date : 30 Aug 2023
//*********************************************************************
// * Software License Agreement (BSD License)
// *
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of the copyright holder nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// *  /* Author: Edwin Ho */
//*********************************************************************

#include "std_msgs/msg/float64.hpp"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

//******************************************************************************************************************************
// transform_point() : transform pose from src_frame to dest_frame
// input :
//	 src_point : src_point to be converted from from_frame to to_frame
//	 src_frame : name of the source frame
//	 dest_frame : name of the destination frame
// output:
//	 src_pose wrt src_pose is tranformed to src_pose wrt dest_frame.
//******************************************************************************************************************************
bool transform_point(std::shared_ptr<rclcpp::Node> node, geometry_msgs::msg::Point &src_point, std::string src_frame, std::string dest_frame)
{
	bool success;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  	
  	tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
	geometry_msgs::msg::TransformStamped t;
	
	// Need to wait until vision node has data
        while(!tf_buffer->canTransform(src_frame, dest_frame, tf2::TimePointZero, std::chrono::milliseconds(10), NULL)) 
        {
            //RCLCPP_INFO(LOGGER, "%s", tf_buffer->allFramesAsString().c_str());
            //RCLCPP_INFO(LOGGER, "waiting %d ms for %s->%s transform to become available", 1000, "tool1", "tool0");
            //RCLCPP_INFO(LOGGER, ".");
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
	//RCLCPP_INFO(LOGGER, "done_1");
	try 
	{
          	t = tf_buffer->lookupTransform(dest_frame, src_frame, tf2::TimePointZero);
          	
          	tf2::doTransform(src_point, src_point, t);
          	
		success  = true;
		
        } 
        catch (const tf2::TransformException & ex)
        {
          	RCLCPP_INFO(node->get_logger(), "transform_point() : Could not transform %s to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
          	success = false;
        }
	return success;
}



//******************************************************************************************************************************
// transform_pose() : transform pose from src_frame to dest_frame
// input :
//	 src_pose : src_pose to be converted from from_frame to to_frame
//	 src_frame : name of the source frame
//	 dest_frame : name of the destination frame
// output:
//	 src_pose wrt src_pose is tranformed to src_pose wrt dest_frame.
//******************************************************************************************************************************
bool transform_pose(std::shared_ptr<rclcpp::Node> node, geometry_msgs::msg::Pose &src_pose, std::string src_frame, std::string dest_frame)
{
	bool success;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  	
  	tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
	geometry_msgs::msg::TransformStamped t;
	
	// Need to wait until vision node has data
        while(!tf_buffer->canTransform(src_frame, dest_frame, tf2::TimePointZero, std::chrono::milliseconds(10), NULL)) 
        {
            RCLCPP_INFO(node->get_logger(), ".");
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
	
	try 
	{
          	t = tf_buffer->lookupTransform(dest_frame, src_frame, tf2::TimePointZero);
          	
          	tf2::doTransform(src_pose, src_pose, t);
          	
		success  = true;
		
        } 
        catch (const tf2::TransformException & ex)
        {
          	RCLCPP_INFO(node->get_logger(), "transform_pose() :Could not transform %s to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
          	success = false;
        }
	return success;
}


//******************************************************************************************************************************
// get_robot_info() : get basic info of robot
// input :
//	 &move_group : pointer to move_group class object
// output:
//	 return basic info of planning frame, end effector link and robot status
//******************************************************************************************************************************
void get_robot_info(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group)
{ 
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // reference 
  // http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  // We can also print joints name of the robot for this planning group.
  std::vector<std::string> joint_names = move_group.getJointNames();
  printf("Joint names : [");
  for (std::string i: joint_names)
   	printf("%s    ", i.c_str());
  printf(" ]\n"); 	
 
  // We can also print joints values of the robot for this planning group.
  std::vector<double> joint_values = move_group.getCurrentJointValues();
  printf("Joint angles : [");
  for (double i: joint_values)
  	printf("%f      ", i);
  printf(" ]\n"); 
  
}

//******************************************************************************************************************************
// rel_single_joint_move() : move a single joint in relative mode
// input :
//	 &move_group : pointer to move_group class object
//       joint_num : index num of the joint (i.e. 0....5)
//       rel_angle : relative joint angle of movement (in degree)
//******************************************************************************************************************************
void rel_single_joint_move(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group, int joint_num, double rel_angle)
{
	std::vector<double> target_joint_angles = move_group.getCurrentJointValues();
	target_joint_angles[joint_num] =  target_joint_angles[joint_num]+rel_angle*(3.14/180.0);
	move_group.setJointValueTarget(target_joint_angles);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  	if(success)
  	{
  	  	move_group.execute(my_plan);
  	  	RCLCPP_INFO(node->get_logger(), "rel_single_joint_move() done");
  	}
  	else
  	{
  		RCLCPP_ERROR(node->get_logger(), "rel_single_joint_move() failed");
  	}
}


//******************************************************************************************************************************
// abs_single_joint_move() : move a single joint in absolute mode
// input :
//	 &move_group : pointer to move_group class object
//       joint_num : index num of the joint (i.e. 0....5)
//       abs_angle : absolute joint angle of movement (in degree)
//******************************************************************************************************************************
void abs_single_joint_move(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group, int joint_num, double abs_angle)
{
	std::vector<double> target_joint_angles = move_group.getCurrentJointValues();
	target_joint_angles[joint_num] =  abs_angle*(3.14/180.0);
	move_group.setJointValueTarget(target_joint_angles);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  	if(success)
  	{
  	  	move_group.execute(my_plan);
  	  	RCLCPP_INFO(node->get_logger(), "abs_single_joint_move() done");
  	}
  	else
  	{
  		RCLCPP_ERROR(node->get_logger(), "abs_single_joint_move() failed");
  	}
}


//******************************************************************************************************************************
// rel_joints_move() : move multiple joints in relative mode
// input :
//	 &move_group : pointer to move_group class object
//       rel_angles : list of relative joint angles [j0, j1, j2, ,j3, j4, j5] (where j0-5 are rel joint angles in degree)
//******************************************************************************************************************************
void rel_joints_move(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> rel_angles)
{
	//chk rel_angles_arr size to be similar to number of joints robot has
	if( rel_angles.size() != 6)
	{
		RCLCPP_ERROR(node->get_logger(), "rel_joints_move : Mismatch size, given input joint : %d instead of 6", (int)rel_angles.size());
		return;
	}
	
	std::vector<double> target_joint_angles = move_group.getCurrentJointValues();
	for(int i=0; i<(int)target_joint_angles.size() ; i++)
	{
		target_joint_angles[i] =  target_joint_angles[i]+rel_angles[i]*(3.14/180.0);
	}
	move_group.setJointValueTarget(target_joint_angles);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  	if(success)
  	{
  	  	move_group.execute(my_plan);
  	  	RCLCPP_INFO(node->get_logger(), "rel_joints_move() done");
  	}
  	else
  	{
  		RCLCPP_ERROR(node->get_logger(), "rel_joints_move() failed");
  	}
}

//******************************************************************************************************************************
// abs_joints_move() : move multiple joints in absolute mode
// input :
//	 &move_group : pointer to move_group class object
//       abs_angles : list of absolutes joint angles [j0, j1, j2, ,j3, j4, j5] (where j0-5 are abs joint angles in degree)
//******************************************************************************************************************************
void abs_joints_move(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group, std::vector<double> abs_angles)
{
	//chk rel_angles_arr size to be similar to number of joints robot has
	if( abs_angles.size() != 6)
	{
		RCLCPP_ERROR(node->get_logger(), "abs_joints_move : Mismatch size, given input joint : %d instead of 6", (int)abs_angles.size());
		return;
	}
	
	std::vector<double> target_joint_angles = move_group.getCurrentJointValues();
	for(int i=0; i<(int)target_joint_angles.size() ; i++)
	{
		target_joint_angles[i] =  abs_angles[i]*(3.14/180.0);
	}
	move_group.setJointValueTarget(target_joint_angles);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  	if(success)
  	{
  	  	move_group.execute(my_plan);
  	  	RCLCPP_INFO(node->get_logger(), "abs_joints_move() done");
  	}
  	else
  	{
  		RCLCPP_ERROR(node->get_logger(), "abs_joints_move() failed");
  	}
}


//******************************************************************************************************************************
// pose_joints_move() : move in pose mode
// input :
//	 &move_group -> pointer to move_group class object
//       string reference_frame: name of goal pose's reference frame [by default : is the base link of the planning group]
//       goal_pose: goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//******************************************************************************************************************************
void pose_move(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group, std::string reference_frame, geometry_msgs::msg::Pose goal_pose)
{	



	  //first check if pose transformation is required
	  //this is done by checking pose_frame_name against planning group's planning frame 
	  //frame_tranformation is not required if pose_frame_name is similar to planning group's planning frame
	  if(move_group.getPlanningFrame() != reference_frame)
	  {
	  	//conduct transformation of pose with reference bk to planning group's planning frame
	  	bool transform_success = transform_pose(node, goal_pose,  reference_frame, move_group.getPlanningFrame());
	  	if(!transform_success)
	  	{
	  		RCLCPP_ERROR(node->get_logger(), "pose_move() : failed. unable to transform Pose from %s to %s", reference_frame.c_str(), move_group.getPlanningFrame().c_str());
	  		return;	  	
	  	}	
	  }

	  move_group.setPoseTarget(goal_pose);
	  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	  if(success)
	  {
	  
	  	move_group.execute(my_plan);
	  	RCLCPP_INFO(node->get_logger(), "pose_move() done");
	  }
	  else
	  {
	  	RCLCPP_ERROR(node->get_logger(), "pose_move() failed");
	  }
}

//******************************************************************************************************************************
// cartesian_joints_move() : move through linearly throughs poses (i.e. waypoints) 
// input :
//	 &move_group -> pointer to move_group class object
//       string reference_frame: name of goal pose's reference frame [by default if not provided, it is the base_link of the planning group]
//       waypoints: list of goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//******************************************************************************************************************************
void cartesian_move(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group, std::string reference_frame, std::vector<geometry_msgs::msg::Pose> waypoints)
{	  
	  //first check if pose transformation is required
	  //this is done by checking waypoints_frame_name against planning group's planning frame 
	  //frame_tranformation is not required if waypoints_frame_name is similar to planning group's planning frame
	  if(move_group.getPlanningFrame() != reference_frame)
	  {
	  	//conduct transformation of pose with reference bk to planning group's planning frame
	  	
	  	for(int i = 0; i < (int)waypoints.size(); i++)
	  	{
		  	bool transform_success = transform_pose(node, waypoints[i],  reference_frame, move_group.getPlanningFrame());
		  	if(!transform_success)
		  	{
		  		RCLCPP_ERROR(node->get_logger(), "cartesian_move() : failed. unable to transform one or more waypoint from %s to %s", 
		  												 reference_frame.c_str(), 
		  												 move_group.getPlanningFrame().c_str());
		  		return;	  	
		  	}
	  	}	
	  }
	  moveit_msgs::msg::RobotTrajectory trajectory;
          const double jump_threshold = 0.0;
          const double eef_step = 0.01;
          double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	  if(fraction == 1.0)
	  {
	   	move_group.execute(trajectory);
	   	RCLCPP_INFO(node->get_logger(), "cartesian_move() done");
	  }
	  else
	  {
	  	RCLCPP_ERROR(node->get_logger(), "cartesian_move: failed : fraction is : %f", fraction);
	  	 
	  }
}

