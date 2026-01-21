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

#include <moveit_msgs/msg/motion_sequence_request.hpp>

#include <moveit_msgs/action/move_group_sequence.hpp>

#include <moveit_msgs/msg/motion_sequence_item.hpp>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;


//******************************************************************************************************************************
// Sequence Class (Pilz Industrial Motion Planner)
//******************************************************************************************************************************
class Sequence
{
  
  public:
            
  	Sequence(std::shared_ptr<rclcpp::Node> node_,moveit::planning_interface::MoveGroupInterface &move_group);
  	
  	moveit_msgs::msg::MotionSequenceItem PTP(std::vector<double> joint_angles, bool relative, double vel_scale, double acc_scale);
  	moveit_msgs::msg::MotionSequenceItem PTP(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale);
	moveit_msgs::msg::MotionSequenceItem LIN(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale);
	moveit_msgs::msg::MotionSequenceItem CIRC(geometry_msgs::msg::Pose goal_pose, geometry_msgs::msg::Point ic_point, std::string  ic_type, std::string reference_frame, bool relative, double vel_scale, double acc_scale);
  	
 	void append(moveit_msgs::msg::MotionSequenceItem mot_item, double blend_radius);
 	void clear();
 	moveit_msgs::msg::MotionSequenceRequest mot_req;
 	
 	
  
  private:
  	rclcpp::Logger LOGGER = rclcpp::get_logger("Pilz.hpp (Sequence) : ");
  	moveit::planning_interface::MoveGroupInterface *move_grp;
  	std::shared_ptr<rclcpp::Node> node;
	bool transform_point(geometry_msgs::msg::Point &src_point, std::string src_frame, std::string dest_frame);
	bool transform_pose(geometry_msgs::msg::Pose &src_pose, std::string src_frame, std::string dest_frame);
	bool pilz_relative_pose_tf(geometry_msgs::msg::Pose &goal_pose, std::string reference_frame);
	bool pilz_relative_point_tf(geometry_msgs::msg::Point &point, std::string reference_frame);
	
	double allowed_planning_time =  1.5;
	double tol_pose = 0.01;
	double tol_angle = 0.01;

	
  		
};


//******************************************************************************************************************************
// Sequence Class constructor
//******************************************************************************************************************************
Sequence::Sequence(std::shared_ptr<rclcpp::Node> node_, moveit::planning_interface::MoveGroupInterface &move_group)
{
	move_grp = &move_group;
	node = node_;
}


//******************************************************************************************************************************
// ptp() sequence planning: move multiple joints in pilz_ptp mode (joint Space)
// input :
//       joint_angles : list of joint angles [j0, j1, j2, ,j3, j4, j5] (where j0-5 are abs joint angles in radian)
//	 reference_frame : not important  for joint space
//	 relative : true or false (if false - joints angles are taken as absolute)
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 to 1.0)
//******************************************************************************************************************************
moveit_msgs::msg::MotionSequenceItem  Sequence::PTP(std::vector<double> joint_angles, bool relative, double vel_scale, double acc_scale)
{
	moveit_msgs::msg::MotionSequenceItem mot_item;
	
	//Setup a joint space goal (PTP)
	mot_item.req.group_name = move_grp->getName();
	mot_item.req.allowed_planning_time = allowed_planning_time;
	mot_item.req.max_velocity_scaling_factor = vel_scale;  
	mot_item.req.max_acceleration_scaling_factor = acc_scale;  
	
	mot_item.req.pipeline_id = "pilz_industrial_motion_planner";
	mot_item.req.planner_id = "PTP"; 
	
	//chk joint_angles size to be similar to number of joints robot has
	if(joint_angles.size() != 7)
	{
		RCLCPP_ERROR(LOGGER, "pilz_Sequence_ptp_joint : Mismatch size, given input joint : %d instead of 6", (int)joint_angles.size());
	}
	
	
	
	std::vector<double> target_joint_angles = move_grp->getCurrentJointValues();
	for(int i=0; i<(int)target_joint_angles.size() ; i++)
	{
		if (relative)
		{
			target_joint_angles[i] =  target_joint_angles[i]+joint_angles[i];
		}
		else
		{
			target_joint_angles[i] =  joint_angles[i];
		}
	}
	
	
       
        const moveit::core::JointModelGroup* joint_model_group = move_grp->getCurrentState()->getJointModelGroup(move_grp->getName());
	moveit::core::RobotState goal_state = *move_grp->getCurrentState();
	goal_state.setJointGroupPositions(joint_model_group, target_joint_angles);
	moveit_msgs::msg::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
	mot_item.req.goal_constraints.clear();
	mot_item.req.goal_constraints.push_back(joint_goal);
  
	return mot_item;
}

//******************************************************************************************************************************
// PTP() sequence planning : move to goal pose in pilz_ptp mode (cartesian Space)
// input :
//       goal_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 reference_frame : name of goal pose's reference frame
//	 relative : true or false 
//		    if true: 
//			     1) the pose will be with reference from end-effector frame.
//			     2) orientation of the pose will be reference to that stated by reference_frame
//		    if false: 
//			     1) the pose will be with reference to reference_frame
//			     2) orientation of the pose will be reference to that state by reference frame
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 - 1.0)
//******************************************************************************************************************************
moveit_msgs::msg::MotionSequenceItem  Sequence::PTP(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale)
{
	moveit_msgs::msg::MotionSequenceItem mot_item;
	
	geometry_msgs::msg::Pose pose_tmp;
  	geometry_msgs::msg::PoseStamped pose;
  
  
  
  	tf2::Quaternion q;
  
  	//Setup a pose goal (PTP)
  	mot_item.req.group_name = move_grp->getName();
  	mot_item.req.allowed_planning_time = allowed_planning_time;
  	mot_item.req.max_velocity_scaling_factor = vel_scale;
  	mot_item.req.max_acceleration_scaling_factor = acc_scale;    
  	mot_item.req.pipeline_id = "pilz_industrial_motion_planner";
  	mot_item.req.planner_id = "PTP"; 

	if(relative)
	{
		if(pilz_relative_pose_tf(goal_pose, reference_frame))
		{
			pose.header.frame_id = move_grp->getPlanningFrame();
	  		pose.pose = goal_pose;
	  
	  		std::vector<double> tolerance_pose(3, tol_pose);
  	  		std::vector<double> tolerance_angle(3, tol_angle);
	  		moveit_msgs::msg::Constraints seq_pose_goal = kinematic_constraints::constructGoalConstraints(move_grp->getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	  		
	  		mot_item.req.goal_constraints.clear();
	  		mot_item.req.goal_constraints.push_back(seq_pose_goal);
			
			
			
		}
		else
		{
			RCLCPP_ERROR(LOGGER, "Sequence PTP pose failed. Relative Pose Tranform failed.");
		}
	}
	else
	{
		if(reference_frame != move_grp->getPlanningFrame())
		{
			bool transform_success = transform_pose(goal_pose,  reference_frame, move_grp->getPlanningFrame() );
			if(transform_success)
			{
				
				pose.header.frame_id = move_grp->getPlanningFrame();
	  			pose.pose = goal_pose;
	  
	  			std::vector<double> tolerance_pose(3, tol_pose);
  	  			std::vector<double> tolerance_angle(3, tol_angle);
	  			moveit_msgs::msg::Constraints seq_pose_goal = kinematic_constraints::constructGoalConstraints(move_grp->getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	  
	  			mot_item.req.goal_constraints.clear();
	  			mot_item.req.goal_constraints.push_back(seq_pose_goal);
				//move_grp->setPlanningPipelineId(default_planning_pipeline);
		  		//move_grp->setPlannerId(default_planning_id ); 
				//return;	  	
			}
			else
			{
				RCLCPP_ERROR(LOGGER, "Sequence PTP pose failed. Pose Transform failed.");
			}
		}	
	}

	
	
	return mot_item;
	
}


//******************************************************************************************************************************
// LIN() sequence planning: move to goal pose in pilz_lin mode (cartesian Space)
// input :
//	 &move_group : pointer to move_group class object
//       goal_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 reference_frame : name of goal pose's reference frame
//	 relative : true or false 
//		    if true: 
//			     1) the pose will be with reference from end-effector frame.
//			     2) orientation of the pose will be reference to that stated by reference_frame
//		    if false: 
//			     1) the pose will be with reference to reference_frame
//			     2) orientation of the pose will be reference to that state by reference frame
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 - 1.0)
//******************************************************************************************************************************
moveit_msgs::msg::MotionSequenceItem  Sequence::LIN(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale)
{
	moveit_msgs::msg::MotionSequenceItem mot_item;
	
	geometry_msgs::msg::Pose pose_tmp;
  	geometry_msgs::msg::PoseStamped pose;
  
  
  
  	tf2::Quaternion q;
  
  	//Setup a pose goal (LIN)
  	mot_item.req.group_name = move_grp->getName();
  	mot_item.req.allowed_planning_time = allowed_planning_time;
  	mot_item.req.max_velocity_scaling_factor = vel_scale;
  	mot_item.req.max_acceleration_scaling_factor = acc_scale;    
  	mot_item.req.pipeline_id = "pilz_industrial_motion_planner";
  	mot_item.req.planner_id = "LIN"; 

	if(relative)
	{
		if(pilz_relative_pose_tf(goal_pose, reference_frame))
		{
			pose.header.frame_id = move_grp->getPlanningFrame();
	  		pose.pose = goal_pose;
	  
	  		std::vector<double> tolerance_pose(3, tol_pose);
  	  		std::vector<double> tolerance_angle(3, tol_angle);
	  		moveit_msgs::msg::Constraints seq_pose_goal = kinematic_constraints::constructGoalConstraints(move_grp->getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	  		
	  		mot_item.req.goal_constraints.clear();
	  		mot_item.req.goal_constraints.push_back(seq_pose_goal);
			
			
			
		}
		else
		{
			RCLCPP_ERROR(LOGGER, "Sequence LIN() failed. Relative Pose Tranform failed.");
		}
	}
	else
	{
		if(reference_frame != move_grp->getPlanningFrame())
		{
			bool transform_success = transform_pose(goal_pose,  reference_frame, move_grp->getPlanningFrame() );
			if(transform_success)
			{
				
				pose.header.frame_id = move_grp->getPlanningFrame();
	  			pose.pose = goal_pose;
	  
	  			std::vector<double> tolerance_pose(3, tol_pose);
  	  			std::vector<double> tolerance_angle(3, tol_angle);
	  			moveit_msgs::msg::Constraints seq_pose_goal = kinematic_constraints::constructGoalConstraints(move_grp->getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	  
	  			mot_item.req.goal_constraints.clear();
	  			mot_item.req.goal_constraints.push_back(seq_pose_goal);
				//move_grp->setPlanningPipelineId(default_planning_pipeline);
		  		//move_grp->setPlannerId(default_planning_id ); 
				//return;	  	
			}
			else
			{
				RCLCPP_ERROR(LOGGER, "Sequence LIN() failed. Pose Transform failed.");
			}
		}	
	}
	return mot_item;
}


//******************************************************************************************************************************
// CIRC() sequence planning : move arc in pilz_circ mode (cartesian Space)
// input :s
//       goal_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 ic_point : interim or center point parameter (i.e. position(x,y,z)
//	 ic_type : "Interim" or "center"
//	 reference_frame : name of goal pose's reference frame
//	 relative : true or false 
//		    if true: 
//			     1) the pose will be with reference from end-effector frame.
//			     2) orientation of the pose will be reference to that stated by reference_frame
//		    if false: 
//			     1) the pose will be with reference to reference_frame
//			     2) orientation of the pose will be reference to that state by reference frame
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 - 1.0)
//******************************************************************************************************************************
moveit_msgs::msg::MotionSequenceItem  Sequence::CIRC(geometry_msgs::msg::Pose goal_pose, geometry_msgs::msg::Point ic_point, std::string  ic_type, std::string reference_frame, bool relative, double vel_scale, double acc_scale)
{
	moveit_msgs::msg::MotionSequenceItem mot_item;
	
	geometry_msgs::msg::Pose pose_tmp;
  	geometry_msgs::msg::PoseStamped pose;
  	geometry_msgs::msg::Pose ic_pose;
	
	mot_item.req.group_name = move_grp->getName();
	mot_item.req.allowed_planning_time = allowed_planning_time;
  	mot_item.req.max_velocity_scaling_factor = vel_scale;
  	mot_item.req.max_acceleration_scaling_factor = acc_scale;    
	mot_item.req.pipeline_id = "pilz_industrial_motion_planner";
	mot_item.req.planner_id = "CIRC"; // "LIN" also fails
	
		
	if(ic_type != "interim" && ic_type != "center")
  	{
  		RCLCPP_ERROR(LOGGER, "Sequence CIRC() failed. Unknown goal type defined %s (type neither interim or center)", ic_type.c_str());
 
  	}
  	else
  	{
  		if (relative)
  		{
	  		if(pilz_relative_pose_tf(goal_pose, reference_frame))
	  		{
	  			
	  			if(pilz_relative_point_tf(ic_point, reference_frame))
		  		{
		  			pose.header.frame_id = move_grp->getPlanningFrame();
		  			pose.pose = goal_pose;
		  
		  			std::vector<double> tolerance_pose(3, tol_pose);
	  	  			std::vector<double> tolerance_angle(3, tol_angle);
		  			moveit_msgs::msg::Constraints seq_pose_goal = kinematic_constraints::constructGoalConstraints(move_grp->getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	  				moveit_msgs::msg::PositionConstraint posc;
	  				posc.header.frame_id = move_grp->getPlanningFrame();
	  				posc.link_name = move_grp->getEndEffectorLink();
	
	  				geometry_msgs::msg::Pose ic_pose;
	  				tf2::Quaternion q;
  	  				q.setRPY(0.0, 0.0, 0.0);
  	  				ic_pose.position.x = ic_point.x;
					ic_pose.position.y = ic_point.y;
					ic_pose.position.z = ic_point.z;
					ic_pose.orientation.x = q.x(); 
					ic_pose.orientation.y = q.y(); 
					ic_pose.orientation.z = q.z(); 
					ic_pose.orientation.w = q.w(); 
					
					posc.constraint_region.primitive_poses.push_back(ic_pose);
		  			moveit_msgs::msg::Constraints path_constraint;
		  			path_constraint.name = ic_type;
		  			path_constraint.position_constraints.push_back(posc);
		  
		  			mot_item.req.goal_constraints.clear();
		  			mot_item.req.goal_constraints.push_back(seq_pose_goal);
		  			mot_item.req.path_constraints = path_constraint;

		  		}
		  		else
		  		{
		  			RCLCPP_ERROR(LOGGER, "Sequence CIRC() failed. pilz_relative_point_tf errror");
		  		}
	  		}
	  		else
	  		{
	  			RCLCPP_ERROR(LOGGER, "Sequence CIRC() failed. pilz_relative_pose_tf errror");
	  		}
	
	  	}
		else
  		{
	  		if(reference_frame != move_grp->getPlanningFrame())
			{
				bool transform_success = transform_pose(goal_pose,  reference_frame, move_grp->getPlanningFrame() );
				if(transform_success)
				{
					transform_success = transform_point(ic_point,  reference_frame, move_grp->getPlanningFrame() );
					if(transform_success)
					{
						pose.header.frame_id = move_grp->getPlanningFrame();
			  			pose.pose = goal_pose;
			  
			  			std::vector<double> tolerance_pose(3, tol_pose);
		  	  			std::vector<double> tolerance_angle(3, tol_angle);
			  			moveit_msgs::msg::Constraints seq_pose_goal = kinematic_constraints::constructGoalConstraints(move_grp->getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
		  				moveit_msgs::msg::PositionConstraint posc;
		  				posc.header.frame_id = move_grp->getPlanningFrame();
		  				posc.link_name = move_grp->getEndEffectorLink();
		
		  				geometry_msgs::msg::Pose ic_pose;
		  				tf2::Quaternion q;
	  	  				q.setRPY(0.0, 0.0, 0.0);
	  	  				ic_pose.position.x = ic_point.x;
						ic_pose.position.y = ic_point.y;
						ic_pose.position.z = ic_point.z;
						ic_pose.orientation.x = q.x(); 
						ic_pose.orientation.y = q.y(); 
						ic_pose.orientation.z = q.z(); 
						ic_pose.orientation.w = q.w(); 
						
						posc.constraint_region.primitive_poses.push_back(ic_pose);
			  			moveit_msgs::msg::Constraints path_constraint;
			  			path_constraint.name = ic_type;
			  			path_constraint.position_constraints.push_back(posc);
			  
			  			mot_item.req.goal_constraints.clear();
			  			mot_item.req.goal_constraints.push_back(seq_pose_goal);
			  			mot_item.req.path_constraints = path_constraint;
						
					}
					else
					{
						RCLCPP_ERROR(LOGGER, "Sequence CIRC() failed. transform_point errror");
					}
		
				}
				else
				{
					RCLCPP_ERROR(LOGGER, "Sequence CIRC() failed. transform_pose errror");
				}
				
				
			}
		}		
  	}
	return mot_item;
}


//******************************************************************************************************************************
// append() : for concatenation of motion plans
// input :
//	 mot item : moveit_msgs::MotionSequenceItem mot_item
//	 blend_radius : blending radius in mm (blend_radis for last sequence appended must be zero)
//******************************************************************************************************************************
void  Sequence::append(moveit_msgs::msg::MotionSequenceItem mot_item,  double blend_radius)
{
	mot_item.blend_radius = blend_radius;
	mot_req.items.push_back(mot_item);   //add to the seq
}

//******************************************************************************************************************************
// clear() : for clearing of all motion plans
//******************************************************************************************************************************
void  Sequence::clear()
{
	mot_req.items.clear();   //remove all
}



//******************************************************************************************************************************
// transform_point() : transform pose from src_frame to dest_frame
// input :
//	 src_point : src_point to be converted from from_frame to to_frame
//	 src_frame : name of the source frame
//	 dest_frame : name of the destination frame
// output:
//	 src_pose wrt src_pose is tranformed to src_pose wrt dest_frame.
//******************************************************************************************************************************
bool Sequence::transform_point(geometry_msgs::msg::Point &src_point, std::string src_frame, std::string dest_frame)
{
	bool success;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  	
  	tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
	geometry_msgs::msg::TransformStamped t;
	
	// Need to wait until vision node has data
        while(!tf_buffer->canTransform(src_frame, dest_frame, tf2::TimePointZero, std::chrono::milliseconds(100), NULL)) 
        {
            //RCLCPP_INFO(LOGGER, "%s", tf_buffer->allFramesAsString().c_str());
            //RCLCPP_INFO(LOGGER, "waiting %d ms for %s->%s transform to become available", 1000, "tool1", "tool0");
            RCLCPP_INFO(LOGGER, ".");
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
          	RCLCPP_INFO(LOGGER, "transform_point() : Could not transform %s to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
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
bool Sequence::transform_pose(geometry_msgs::msg::Pose &src_pose, std::string src_frame, std::string dest_frame)
{
	bool success;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  	
  	tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
	geometry_msgs::msg::TransformStamped t;
	
	// Need to wait until vision node has data
        while(!tf_buffer->canTransform(src_frame, dest_frame, tf2::TimePointZero, std::chrono::milliseconds(100), NULL)) 
        {
            RCLCPP_INFO(LOGGER, "%s", tf_buffer->allFramesAsString().c_str());
            //RCLCPP_INFO(LOGGER, "waiting %d ms for %s->%s transform to become available", 1000, "tool1", "tool0");
            RCLCPP_INFO(LOGGER, ".");
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
	//RCLCPP_INFO(LOGGER, "done_1");
	try 
	{
          	t = tf_buffer->lookupTransform(dest_frame, src_frame, tf2::TimePointZero);
          	
          	tf2::doTransform(src_pose, src_pose, t);
          	
		success  = true;
		
        } 
        catch (const tf2::TransformException & ex)
        {
          	RCLCPP_INFO(LOGGER, "transform_pose() :Could not transform %s to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
          	success = false;
        }
	return success;
}

//******************************************************************************************************************************
// pilz_relative_pose_tf() : transform relative pose with respect tcp origin (for pose transform LIN)
// input :
//	 rel_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 reference_frame : name of goal pose's reference frame
//******************************************************************************************************************************
bool Sequence::pilz_relative_pose_tf(geometry_msgs::msg::Pose &goal_pose, std::string reference_frame)
{
	bool result = true;
	geometry_msgs::msg::Pose processing_pose;
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0); 
	processing_pose.position.x = 0.0;
	processing_pose.position.y = 0.0;
	processing_pose.position.z = 0.0;
	processing_pose.orientation.x = q.x();
	processing_pose.orientation.y = q.y();
	processing_pose.orientation.z = q.z();
	processing_pose.orientation.w = q.w();
	
	if(reference_frame != move_grp->getEndEffectorLink())
	{

		//get position of reference frame's origin wrt end effector frame - vector_a
		bool transform_success = transform_pose(processing_pose,  reference_frame, move_grp->getEndEffectorLink());
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform origin from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
			result = false;	
		}
		
		std::vector<double> vector_a = {processing_pose.position.x, processing_pose.position.y, processing_pose.position.z};
		
		if(result)
		{
			
			processing_pose = goal_pose;
			//get position of goal_pose(in reference frame) from end effector frame - vector_b
			transform_success = transform_pose(processing_pose,  reference_frame, move_grp->getEndEffectorLink());
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
				result = false;	  	
			}
		}
		
		std::vector<double> vector_b = {processing_pose.position.x, processing_pose.position.y, processing_pose.position.z};
		
		if(result)
		{
			
			
			//vector_c = vector_b - vector_a
			std::vector<double> vector_c = {vector_b[0]-vector_a[0], vector_b[1]-vector_a[1], vector_b[2]-vector_a[2]};
			
			//process goal_pose to be respect to planning_frame
			goal_pose.position.x = vector_c[0];
			goal_pose.position.y = vector_c[1];
			goal_pose.position.z = vector_c[2];
			goal_pose.orientation.x = q.x();
			goal_pose.orientation.y = q.y();
			goal_pose.orientation.z = q.z();
			goal_pose.orientation.w = q.w();
			
			transform_success = transform_pose(goal_pose,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
				result =  false;	  	
			}
		}
	}
	else //if reference_frame == end effector frame
	{

		bool transform_success = transform_pose(goal_pose,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
			result =  false;	  	
		}
	}
	
	return result;
}

//******************************************************************************************************************************
// pilz_relative_point_tf() : transform relative pose with respect tcp origin (for pose transform LIN)
// input :
//	 point : point parameter in geometry_msgs:msg::Point format (i.e. position(x,y,z))
//	 reference_frame : name of point's reference frame
//******************************************************************************************************************************
bool Sequence::pilz_relative_point_tf(geometry_msgs::msg::Point &point, std::string reference_frame)
{	
	bool result = true;
	geometry_msgs::msg::Point processing_point;
	processing_point.x = 0.0;
	processing_point.y = 0.0;
	processing_point.z = 0.0;

	
	if(reference_frame != move_grp->getEndEffectorLink())
	{
		//get position of reference frame's origin wrt end effector frame - vector_a
		bool transform_success = transform_point(processing_point,  reference_frame, move_grp->getEndEffectorLink());
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_point_tf : failed. unable to transform origin from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
			result = false;	
		}
		
		std::vector<double> vector_a = {processing_point.x, processing_point.y, processing_point.z};
		
		if(result)
		{
			


			processing_point = point;
			//get position of goal_pose(in reference frame) from end effector frame - vector_b
			transform_success = transform_point(processing_point,  reference_frame, move_grp->getEndEffectorLink());
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_point_tf : failed. unable to transform point from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
				result = false;	  	
			}
		}
		
		std::vector<double> vector_b = {processing_point.x, processing_point.y, processing_point.z};
		
		if(result)
		{
			
			
			//vector_c = vector_b - vector_a
			std::vector<double> vector_c = {vector_b[0]-vector_a[0], vector_b[1]-vector_a[1], vector_b[2]-vector_a[2]};
			
			//process goal_pose to be respect to planning_frame
			point.x = vector_c[0];
			point.y = vector_c[1];
			point.z = vector_c[2];

			
			transform_success = transform_point(point,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_point_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
				result =  false;	  	
			}
		}
	}
	else //if reference_frame == end effector frame
	{
		bool transform_success = transform_point(point,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
			result =  false;	  	
		}
	}
	
	return result;
}






//******************************************************************************************************************************
// Pilz Class
//******************************************************************************************************************************
class Pilz
{
  public:
	Pilz();
	Pilz(std::shared_ptr<rclcpp::Node> node, moveit::planning_interface::MoveGroupInterface &move_group);
	void PTP(std::vector<double> joint_angles, bool relative, double vel_scale, double acc_scale);
	void PTP(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale);
	void LIN(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale);
	void CIRC(geometry_msgs::msg::Pose goal_pose, 
		  geometry_msgs::msg::Point ic_point,
	          std::string  ic_type,
	          std::string reference_frame, 
	          bool relative, 
	          double vel_scale, 
	          double acc_scale);
	bool SEQ_MOVE(Sequence seq);          
	
  
  private:
  	rclcpp::Logger LOGGER = rclcpp::get_logger("Pilz.hpp");
  	std::shared_ptr<rclcpp::Node> node;
  	moveit::planning_interface::MoveGroupInterface *move_grp;
  	std::string default_planning_pipeline;
	std::string default_planning_id; 
  	bool transform_point(geometry_msgs::msg::Point &src_point, std::string src_frame, std::string dest_frame);
  	bool transform_pose(geometry_msgs::msg::Pose &src_pose, std::string src_frame, std::string dest_frame);
        bool pilz_relative_pose_tf(geometry_msgs::msg::Pose &goal_pose, std::string reference_frame);
	bool pilz_relative_point_tf(geometry_msgs::msg::Point &point, std::string reference_frame);
	rclcpp_action::Client<MoveGroupSequence>::SharedPtr action_client;
};

//******************************************************************************************************************************
// Pilz Class constructor
//******************************************************************************************************************************
Pilz::Pilz()
{

};

//******************************************************************************************************************************
// Pilz Class constructor
//******************************************************************************************************************************
Pilz::Pilz(std::shared_ptr<rclcpp::Node> node_, moveit::planning_interface::MoveGroupInterface &move_group)
{
	move_grp = &move_group;
	node = node_;
	default_planning_pipeline = move_grp->getPlanningPipelineId();
	default_planning_id = move_grp->getPlanningPipelineId();
	
	//create action client for sequence
	action_client = rclcpp_action::create_client<MoveGroupSequence>(node, "/sequence_move_group");
	
}


//******************************************************************************************************************************
// transform_point() : transform pose from src_frame to dest_frame
// input :
//	 src_point : src_point to be converted from from_frame to to_frame
//	 src_frame : name of the source frame
//	 dest_frame : name of the destination frame
// output:
//	 src_pose wrt src_pose is tranformed to src_pose wrt dest_frame.
//******************************************************************************************************************************
bool Pilz::transform_point(geometry_msgs::msg::Point &src_point, std::string src_frame, std::string dest_frame)
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
            RCLCPP_INFO(LOGGER, ".");
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
	//RCLCPP_INFO(LOGGER, "done_1");
	try 
	{
          	t = tf_buffer->lookupTransform(dest_frame, src_frame, tf2::TimePointZero);
          	/*
          	RCLCPP_INFO(LOGGER, "pose3.x : %f", src_pose.position.x);
		RCLCPP_INFO(LOGGER, "pose3.y : %f", src_pose.position.y);
		RCLCPP_INFO(LOGGER, "pose3.z : %f", src_pose.position.z);
		RCLCPP_INFO(LOGGER, "pose3.ox : %f",src_pose.orientation.x);
		RCLCPP_INFO(LOGGER, "pose3.oy : %f",src_pose.orientation.y);
		RCLCPP_INFO(LOGGER, "pose3.oz : %f",src_pose.orientation.z);
		RCLCPP_INFO(LOGGER, "pose3.ow : %f",src_pose.orientation.w);
		*/
          	tf2::doTransform(src_point, src_point, t);
          	/*
		RCLCPP_INFO(LOGGER, "pose3.x : %f", src_pose.position.x);
		RCLCPP_INFO(LOGGER, "pose3.y : %f", src_pose.position.y);
		RCLCPP_INFO(LOGGER, "pose3.z : %f", src_pose.position.z);
		RCLCPP_INFO(LOGGER, "pose3.ox : %f",src_pose.orientation.x);
		RCLCPP_INFO(LOGGER, "pose3.oy : %f",src_pose.orientation.y);
		RCLCPP_INFO(LOGGER, "pose3.oz : %f",src_pose.orientation.z);
		RCLCPP_INFO(LOGGER, "pose3.ow : %f",src_pose.orientation.w);
		*/
		success  = true;
		
        } 
        catch (const tf2::TransformException & ex)
        {
          	RCLCPP_INFO(LOGGER, "transform_point() : Could not transform %s to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
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
bool Pilz::transform_pose(geometry_msgs::msg::Pose &src_pose, std::string src_frame, std::string dest_frame)
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
            RCLCPP_INFO(LOGGER, ".");
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
	//RCLCPP_INFO(LOGGER, "done_1");
	try 
	{
          	t = tf_buffer->lookupTransform(dest_frame, src_frame, tf2::TimePointZero);
          	/*
          	RCLCPP_INFO(LOGGER, "pose3.x : %f", src_pose.position.x);
		RCLCPP_INFO(LOGGER, "pose3.y : %f", src_pose.position.y);
		RCLCPP_INFO(LOGGER, "pose3.z : %f", src_pose.position.z);
		RCLCPP_INFO(LOGGER, "pose3.ox : %f",src_pose.orientation.x);
		RCLCPP_INFO(LOGGER, "pose3.oy : %f",src_pose.orientation.y);
		RCLCPP_INFO(LOGGER, "pose3.oz : %f",src_pose.orientation.z);
		RCLCPP_INFO(LOGGER, "pose3.ow : %f",src_pose.orientation.w);
		*/
          	tf2::doTransform(src_pose, src_pose, t);
          	/*
		RCLCPP_INFO(LOGGER, "pose3.x : %f", src_pose.position.x);
		RCLCPP_INFO(LOGGER, "pose3.y : %f", src_pose.position.y);
		RCLCPP_INFO(LOGGER, "pose3.z : %f", src_pose.position.z);
		RCLCPP_INFO(LOGGER, "pose3.ox : %f",src_pose.orientation.x);
		RCLCPP_INFO(LOGGER, "pose3.oy : %f",src_pose.orientation.y);
		RCLCPP_INFO(LOGGER, "pose3.oz : %f",src_pose.orientation.z);
		RCLCPP_INFO(LOGGER, "pose3.ow : %f",src_pose.orientation.w);
		*/
		success  = true;
		
        } 
        catch (const tf2::TransformException & ex)
        {
          	RCLCPP_INFO(LOGGER, "transform_pose() :Could not transform %s to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
          	success = false;
        }
	return success;
}


//******************************************************************************************************************************
// pilz_relative_pose_tf() : transform relative pose with respect tcp origin (for pose transform LIN)
// input :
//	 rel_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 reference_frame : name of goal pose's reference frame
//******************************************************************************************************************************
bool Pilz::pilz_relative_pose_tf(geometry_msgs::msg::Pose &goal_pose, std::string reference_frame)
{
	bool result = true;
	geometry_msgs::msg::Pose processing_pose;
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0); 
	processing_pose.position.x = 0.0;
	processing_pose.position.y = 0.0;
	processing_pose.position.z = 0.0;
	processing_pose.orientation.x = q.x();
	processing_pose.orientation.y = q.y();
	processing_pose.orientation.z = q.z();
	processing_pose.orientation.w = q.w();
	
	if(reference_frame != move_grp->getEndEffectorLink())
	{

		//get position of reference frame's origin wrt end effector frame - vector_a
		bool transform_success = transform_pose(processing_pose,  reference_frame, move_grp->getEndEffectorLink());
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform origin from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
			result = false;	
		}
		
		std::vector<double> vector_a = {processing_pose.position.x, processing_pose.position.y, processing_pose.position.z};
		
		if(result)
		{
			
			processing_pose = goal_pose;
			//get position of goal_pose(in reference frame) from end effector frame - vector_b
			transform_success = transform_pose(processing_pose,  reference_frame, move_grp->getEndEffectorLink());
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
				result = false;	  	
			}
		}
		
		std::vector<double> vector_b = {processing_pose.position.x, processing_pose.position.y, processing_pose.position.z};
		
		if(result)
		{
			
			
			//vector_c = vector_b - vector_a
			std::vector<double> vector_c = {vector_b[0]-vector_a[0], vector_b[1]-vector_a[1], vector_b[2]-vector_a[2]};
			
			//process goal_pose to be respect to planning_frame
			goal_pose.position.x = vector_c[0];
			goal_pose.position.y = vector_c[1];
			goal_pose.position.z = vector_c[2];
			goal_pose.orientation.x = q.x();
			goal_pose.orientation.y = q.y();
			goal_pose.orientation.z = q.z();
			goal_pose.orientation.w = q.w();
			
			transform_success = transform_pose(goal_pose,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
				result =  false;	  	
			}
		}
	}
	else //if reference_frame == end effector frame
	{

		bool transform_success = transform_pose(goal_pose,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
			result =  false;	  	
		}
	}
	
	return result;
}

//******************************************************************************************************************************
// pilz_relative_point_tf() : transform relative pose with respect tcp origin (for pose transform LIN)
// input :
//	 point : point parameter in geometry_msgs:msg::Point format (i.e. position(x,y,z))
//	 reference_frame : name of point's reference frame
//******************************************************************************************************************************
bool Pilz::pilz_relative_point_tf(geometry_msgs::msg::Point &point, std::string reference_frame)
{	
	bool result = true;
	geometry_msgs::msg::Point processing_point;
	processing_point.x = 0.0;
	processing_point.y = 0.0;
	processing_point.z = 0.0;

	
	if(reference_frame != move_grp->getEndEffectorLink())
	{
		//get position of reference frame's origin wrt end effector frame - vector_a
		bool transform_success = transform_point(processing_point,  reference_frame, move_grp->getEndEffectorLink());
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_point_tf : failed. unable to transform origin from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
			result = false;	
		}
		
		std::vector<double> vector_a = {processing_point.x, processing_point.y, processing_point.z};
		
		if(result)
		{
			


			processing_point = point;
			//get position of goal_pose(in reference frame) from end effector frame - vector_b
			transform_success = transform_point(processing_point,  reference_frame, move_grp->getEndEffectorLink());
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_point_tf : failed. unable to transform point from %s to %s", reference_frame.c_str(), move_grp->getEndEffectorLink().c_str());
				result = false;	  	
			}
		}
		
		std::vector<double> vector_b = {processing_point.x, processing_point.y, processing_point.z};
		
		if(result)
		{
			
			
			//vector_c = vector_b - vector_a
			std::vector<double> vector_c = {vector_b[0]-vector_a[0], vector_b[1]-vector_a[1], vector_b[2]-vector_a[2]};
			
			//process goal_pose to be respect to planning_frame
			point.x = vector_c[0];
			point.y = vector_c[1];
			point.z = vector_c[2];

			
			transform_success = transform_point(point,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "pilz_relative_point_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
				result =  false;	  	
			}
		}
	}
	else //if reference_frame == end effector frame
	{
		bool transform_success = transform_point(point,  move_grp->getEndEffectorLink(), move_grp->getPlanningFrame() );
		if(!transform_success)
		{
			RCLCPP_ERROR(LOGGER, "pilz_relative_pose_tf : failed. unable to transform pose from %s to %s", move_grp->getEndEffectorLink().c_str(), move_grp->getPlanningFrame().c_str());
			result =  false;	  	
		}
	}
	
	return result;
}





//******************************************************************************************************************************
// ptp() : move multiple joints in pilz_ptp mode (joint Space)
// input :
//       joint_angles : list of joint angles [j0, j1, j2, ,j3, j4, j5] (where j0-5 are abs joint angles in radian)
//	 reference_frame : not important  for joint space
//	 relative : true or false (if false - joints angles are taken as absolute)
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 to 1.0)
//******************************************************************************************************************************
void Pilz::PTP(std::vector<double> joint_angles, bool relative, double vel_scale, double acc_scale)
{
	//set planner to pilz industrial motion planner
	move_grp->setPlanningPipelineId("pilz_industrial_motion_planner");
  	// set pilz_industrial_motion planner to PTP mode
  	move_grp->setPlannerId("PTP"); 
		
	//chk joint_angles size to be similar to number of joints robot has
	if(joint_angles.size() != 7)
	{
		RCLCPP_ERROR(LOGGER, "pilz_ptp_joint : Mismatch size, given input joint : %d instead of 6", (int)joint_angles.size());
		
		//set original planner and planner_id
		move_grp->setPlanningPipelineId(default_planning_pipeline);
	  	move_grp->setPlannerId(default_planning_id ); 
		return;
	}
	
	std::vector<double> target_joint_angles = move_grp->getCurrentJointValues();
	for(int i=0; i<(int)target_joint_angles.size() ; i++)
	{
		if (relative)
		{
			target_joint_angles[i] =  target_joint_angles[i]+joint_angles[i];
		}
		else
		{
			target_joint_angles[i] =  joint_angles[i];
		}
	}
	
	
	move_grp->setJointValueTarget(target_joint_angles);
	
	//set velocity and acceleration scale
	move_grp->setMaxVelocityScalingFactor(vel_scale);
  	move_grp->setMaxAccelerationScalingFactor(acc_scale);
  	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	bool success = (move_grp->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  	if(success)
  	{
  	  	move_grp->execute(my_plan);
  	  	RCLCPP_INFO(LOGGER, "PTP_joint() done");
  	}
  	else
  	{
  		RCLCPP_ERROR(LOGGER, "PTP_joint() failed");
  	}
  	
  	//set original planner and planner_id
	move_grp->setPlanningPipelineId(default_planning_pipeline);
	move_grp->setPlannerId(default_planning_id ); 
}


//******************************************************************************************************************************
// PTP() : move to goal pose in pilz_ptp mode (cartesian Space)
// input :
//       goal_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 reference_frame : name of goal pose's reference frame
//	 relative : true or false 
//		    if true: 
//			     1) the pose will be with reference from end-effector frame.
//			     2) orientation of the pose will be reference to that stated by reference_frame
//		    if false: 
//			     1) the pose will be with reference to reference_frame
//			     2) orientation of the pose will be reference to that state by reference frame
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 - 1.0)
//******************************************************************************************************************************
void Pilz::PTP(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale)
{
	//set planner to pilz industrial motion planner
	move_grp->setPlanningPipelineId("pilz_industrial_motion_planner");
  	// set pilz_industrial_motion planner to PTP mode
  	move_grp->setPlannerId("PTP"); 
	

	if(relative)
	{
		if(!pilz_relative_pose_tf(goal_pose, reference_frame))
		{
			RCLCPP_ERROR(LOGGER, "PTP_pose() failed");
			//set original planner and planner_id
			move_grp->setPlanningPipelineId(default_planning_pipeline);
		  	move_grp->setPlannerId(default_planning_id ); 
			return;
		}
	}
	else
	{
		if(reference_frame != move_grp->getPlanningFrame())
		{
			bool transform_success = transform_pose(goal_pose,  reference_frame, move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "PTP_pose() failed");
				move_grp->setPlanningPipelineId(default_planning_pipeline);
		  		move_grp->setPlannerId(default_planning_id ); 
				return;	  	
			}
		}	
	}
	  
	move_grp->setPoseTarget(goal_pose);
	
	//set velocity and acceleration scale
	move_grp->setMaxVelocityScalingFactor(vel_scale);
  	move_grp->setMaxAccelerationScalingFactor(acc_scale);
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_grp->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if(success)
	{
	  
	  	move_grp->execute(my_plan);
	  	RCLCPP_INFO(LOGGER, "PTP_pose() done");
	}
	else
	{
	 	RCLCPP_ERROR(LOGGER, "PTP_pose() failed");
	}
	//set original planner and planner_id
	move_grp->setPlanningPipelineId(default_planning_pipeline);
	move_grp->setPlannerId(default_planning_id ); 	
}

//******************************************************************************************************************************
// LIN() : move to goal pose in pilz_lin mode (cartesian Space)
// input :
//	 &move_group : pointer to move_group class object
//       goal_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 reference_frame : name of goal pose's reference frame
//	 relative : true or false 
//		    if true: 
//			     1) the pose will be with reference from end-effector frame.
//			     2) orientation of the pose will be reference to that stated by reference_frame
//		    if false: 
//			     1) the pose will be with reference to reference_frame
//			     2) orientation of the pose will be reference to that state by reference frame
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 - 1.0)
//******************************************************************************************************************************
void Pilz::LIN(geometry_msgs::msg::Pose goal_pose, std::string reference_frame, bool relative, double vel_scale, double acc_scale)
{	

	//set planner to pilz industrial motion planner
	move_grp->setPlanningPipelineId("pilz_industrial_motion_planner");
  	// set pilz_industrial_motion planner to LIN mode
  	move_grp->setPlannerId("LIN"); 
	

	if(relative)
	{
		if(!pilz_relative_pose_tf(goal_pose, reference_frame))
		{
			RCLCPP_ERROR(LOGGER, "LIN() failed");
			//set original planner and planner_id
			move_grp->setPlanningPipelineId(default_planning_pipeline);
			move_grp->setPlannerId(default_planning_id ); 
			return;
		}

	}
	else
	{
		if(reference_frame != move_grp->getPlanningFrame())
		{
			bool transform_success = transform_pose(goal_pose,  reference_frame, move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "LIN() failed");
				move_grp->setPlanningPipelineId(default_planning_pipeline);
		  		move_grp->setPlannerId(default_planning_id ); 
				return;	  	
			}

		}

	}
	
	  
	move_grp->setPoseTarget(goal_pose);
	
	//set velocity and acceleration scale
	move_grp->setMaxVelocityScalingFactor(vel_scale);
  	move_grp->setMaxAccelerationScalingFactor(acc_scale);
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_grp->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if(success)
	{
	  
	  	move_grp->execute(my_plan);
	  	RCLCPP_INFO(LOGGER, "LIN() done");
	}
	else
	{
	 	RCLCPP_ERROR(LOGGER, "LIN() failed");
	}
	//set original planner and planner_id
	move_grp->setPlanningPipelineId(default_planning_pipeline);
	move_grp->setPlannerId(default_planning_id ); 
}

//******************************************************************************************************************************
// CIRC() : move arc in pilz_circ mode (cartesian Space)
// input :
//       goal_pose : goal pose parameter in geometry_msgs:msg::Pose format (i.e. position(x,y,z), orientation(x,y,z,w)
//	 ic_point : interim or center point parameter (i.e. position(x,y,z)
//	 ic_type : "Interim" or "center"
//	 reference_frame : name of goal pose's reference frame
//	 relative : true or false 
//		    if true: 
//			     1) the pose will be with reference from end-effector frame.
//			     2) orientation of the pose will be reference to that stated by reference_frame
//		    if false: 
//			     1) the pose will be with reference to reference_frame
//			     2) orientation of the pose will be reference to that state by reference frame
//	 vel_scale : velocity sscale (0.0 - 1.0)
//	 acc_scale : acceleration scale (0.0 - 1.0)
//******************************************************************************************************************************
void Pilz::CIRC(geometry_msgs::msg::Pose goal_pose,
	       geometry_msgs::msg::Point ic_point,
	       std::string  ic_type,
	       std::string reference_frame, 
	       bool relative, 
	       double vel_scale, 
	       double acc_scale)
{
	//set planner to pilz industrial motion planner
	move_grp->setPlanningPipelineId("pilz_industrial_motion_planner");
  	// set pilz_industrial_motion planner to CIRC mode
  	move_grp->setPlannerId("CIRC"); 
  	
  	if(ic_type != "interim" && ic_type != "center")
  	{
  		RCLCPP_ERROR(LOGGER, "CIRC : failed. unknown goal type defined %s (type neither interim or center)", ic_type.c_str());
  		return;
  	}
  	
  	if (relative)
  	{
  		if(!pilz_relative_pose_tf(goal_pose, reference_frame))
  		{
  			RCLCPP_ERROR(LOGGER, "CIRC : failed. pilz_relative_pose_tf errror");
  			//set original planner and planner_id
			move_grp->setPlanningPipelineId(default_planning_pipeline);
			move_grp->setPlannerId(default_planning_id ); 

  			return;
  		}
  		
  		if(!pilz_relative_point_tf(ic_point, reference_frame))
  		{
  			RCLCPP_ERROR(LOGGER, "CIRC : failed. pilz_relative_point_tf errror");
  			//set original planner and planner_id
			move_grp->setPlanningPipelineId(default_planning_pipeline);
			move_grp->setPlannerId(default_planning_id ); 

  			return;
  		}
  	}
  	else
  	{
  		if(reference_frame != move_grp->getPlanningFrame())
		{
			bool transform_success = transform_pose(goal_pose,  reference_frame, move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "CIRC() failed");
				move_grp->setPlanningPipelineId(default_planning_pipeline);
		  		move_grp->setPlannerId(default_planning_id ); 
		  		
				return;	  	
			}
			
			transform_success = transform_point(ic_point,  reference_frame, move_grp->getPlanningFrame() );
			if(!transform_success)
			{
				RCLCPP_ERROR(LOGGER, "CIRC() failed");
				move_grp->setPlanningPipelineId(default_planning_pipeline);
		  		move_grp->setPlannerId(default_planning_id ); 
		  		
				return;	  	
			}
		}	
  	
  	}
  	
  	moveit_msgs::msg::PositionConstraint posc;
	posc.header.frame_id = move_grp->getPlanningFrame();
	posc.link_name = move_grp->getEndEffectorLink();
	
	geometry_msgs::msg::Pose ic_pose;
	tf2::Quaternion q;
  	q.setRPY(0.0, 0.0, 0.0);
	ic_pose.position.x = ic_point.x;
	ic_pose.position.y = ic_point.y;
	ic_pose.position.z = ic_point.z;
	ic_pose.orientation.x = q.x(); 
	ic_pose.orientation.y = q.y(); 
	ic_pose.orientation.z = q.z(); 
	ic_pose.orientation.w = q.w(); 
	
	posc.constraint_region.primitive_poses.push_back(ic_pose);
	moveit_msgs::msg::Constraints constraints;
	constraints.name = ic_type;
	constraints.position_constraints.push_back(posc);
	move_grp->setPathConstraints(constraints);
	move_grp->setPoseTarget(goal_pose);

  	//set velocity and acceleration scale
	move_grp->setMaxVelocityScalingFactor(vel_scale);
  	move_grp->setMaxAccelerationScalingFactor(acc_scale);
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_grp->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if(success)
	{
	  
	  	move_grp->execute(my_plan);
	  	RCLCPP_INFO(LOGGER, "CIRC done");
	}
	else
	{
	 	RCLCPP_ERROR(LOGGER, "CIRC failed");
	}	
	
	move_grp->clearPathConstraints();
	
	//set original planner and planner_id
	move_grp->setPlanningPipelineId(default_planning_pipeline);
	move_grp->setPlannerId(default_planning_id ); 

}


//******************************************************************************************************************************
// SEQ_MOVE() : Execute Sequence plans
// input :
//       seq : Sequence plan of Class Sequence (with /sequence_motion_reques)
//******************************************************************************************************************************
bool Pilz::SEQ_MOVE(Sequence seq)
{
    // 1. Check if Server is there
    if (!action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(LOGGER, " /sequence_move_group Action server not available");
        return false; // <--- Return false (Fehler)
    }
    RCLCPP_INFO(LOGGER, "/sequence_move_group Action server ready!");
    
    // 2. Send Goal
    moveit_msgs::action::MoveGroupSequence_Goal mot_seq_goal;
    mot_seq_goal.request = seq.mot_req;
    
    auto future_goal_handle = action_client->async_send_goal(mot_seq_goal);
    
    // Wait for the goal to be accepted
    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
        return false; // <--- Return false
    }

    // 3. Wait for Result
    auto future_result = action_client->async_get_result(goal_handle); 
    auto result = future_result.get(); 

    // 4. Check Result
    if(result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_ERROR(LOGGER, "Pilz Sequence: not succeed (Code: %d)", (int)result.code);
        return false; // <--- Return false (Planung fehlgeschlagen)
    }
    else
    {
        // Optional: Check MoveIt internal error code to be absolutely sure
        if (result.result->response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
             RCLCPP_ERROR(LOGGER, "Action succeeded but MoveIt Planning failed!");
             return false;
        }

        RCLCPP_INFO(LOGGER, "Pilz Sequence: succeed");
        return true; // <--- Return true (Erfolg!)
    }
}
