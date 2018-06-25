/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Niklas Schaefer */

#include <iimoveit/robot_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <iiwa_msgs/JointPosition.h> 
#include <joint_follower/Coupler.h>
#include <joint_follower/GetJointPositions.h>

//TODO not only use positions, use speed and accelerations too
//TODO try out using moveIt! planning live

namespace joint_follower {


class JointFollower : public iimoveit::RobotInterface {
public:
  JointFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame, double scale_factor, double max_radius, bool rad_input)
      : RobotInterface(node_handle, planning_group, base_frame),
        scale_factor_(scale_factor),
        max_radius_(max_radius),
        max_radius2_(max_radius*max_radius),
				rad_input_(rad_input),
				first_time_(true),
        coupled_(true) {

		if(rad_input_) {
			angle_conversion_ = 1.0;	
		}
		else {
			angle_conversion_ = 3.1416/180.0;
		}

		// use when base pose is given
//		base_pose_.position.x = 0.0; //changed base pose -> base in shoulder
//    base_pose_.position.y = 0.5;
//    base_pose_.position.z = 0.6;
//    base_pose_.orientation.x = -sqrt(2.0)/2.0;
//    base_pose_.orientation.y = 0.0;
//    base_pose_.orientation.z = 0.0;
//    base_pose_.orientation.w = sqrt(2.0)/2.0;

		// use when initial joint positions are given
		iiwa_initial_joint_positions_.joint_names.resize(7);
		iiwa_initial_joint_positions_.joint_names = RobotInterface::getJointNames();
		iiwa_initial_joint_positions_.points.resize(1);
		iiwa_initial_joint_positions_.points[0].positions.resize(7);
    // Anthropomorphic. Mirrored
		// iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * -1.0 * -30.97;
		// iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (-1.0 * 18.34 + 90.0);
		// iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -21.67;
		// iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -1.0 * -57.57;
		// iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (59.36 - 90.0); 
		// iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * (-1.0 * -4.63 + 90.0); 
		// iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

    // Initial positions. Anthropomorphic. Workspace to the side. Not mirrored
    iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * 18.34;
    iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (-30.97 + 90.0);
    iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * (-1.0 * -21.67 - 90.0);
    iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -1.0 * -57.57;
    iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (-1.0 * -110.0 - 90.0); 
    iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * (0.0 + 90.0); 
    iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

    // Impedance video
    // iiwa_initial_joint_positions_.points[0].positions[0] = 0.0955010280013;
    // iiwa_initial_joint_positions_.points[0].positions[1] = 1.05284523964;
    // iiwa_initial_joint_positions_.points[0].positions[2] = -0.927038908005;
    // iiwa_initial_joint_positions_.points[0].positions[3] = 1.33675003052;
    // iiwa_initial_joint_positions_.points[0].positions[4] = 0.237600877881; 
    // iiwa_initial_joint_positions_.points[0].positions[5] = 1.35565376282; 
    // iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

    // iiwa_initial_joint_positions_.points[0].positions[0] = 0.158099394662;
    // iiwa_initial_joint_positions_.points[0].positions[1] = 1.04273171673;
    // iiwa_initial_joint_positions_.points[0].positions[2] = -0.930604292196;
    // iiwa_initial_joint_positions_.points[0].positions[3] = 1.32590148519;
    // iiwa_initial_joint_positions_.points[0].positions[4] = 0.254567650553; 
    // iiwa_initial_joint_positions_.points[0].positions[5] = 1.36908613857; 
    // iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

    // // Initial positions for psychological experiment
    // iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * 90.0;
    // iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * 0.0;
    // iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * 0.0;
    // iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -45.0;
    // iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * 90.0; 
    // iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * - 90.0; 
    // iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

		// initial_positions for pick and place, iiwa mounted on tabletop
//		iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * -8.2299;
//		iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (-1.0 * -3.0850 + 90.0);
//		iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -1.0 * -13.7768;
//		iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -1.0 * -52.9783;
//		iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (-1.0 * 26.3020 - 90.0); 
//		iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * 32.7907; 
//		iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

		// pick and place test
//		iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * 0.0;
//		iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (-1.0 * 0.0 + 45.0);
//		iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -1.0 * -0.0;
//		iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * (-1.0 * 0.0 - 90.0);
//		iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (-1.0 * 0.0 - 0.0); 
//		iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * (0.0 - 45.0); 
//		iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

//		iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * 0.0;
//		iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (0.0 - 90.0);
//		iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -1.0 * 0.0;
//		iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * 0.0;
//		iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (-1.0 * 0.0 + 90.0); 
//		iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * 0.0; 
//		iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

    // // Initial positions for parabola movement
    // iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * 73.36;
    // iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * 64.80;
    // iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -47.49;
    // iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -42.24;
    // iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * -54.34; 
    // iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * 43.34; 
    // iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 98.43;

		mcs_initial_joint_positions_.joint_names.resize(7);
		mcs_initial_joint_positions_.joint_names = RobotInterface::getJointNames();
		mcs_initial_joint_positions_.points.resize(1);
		mcs_initial_joint_positions_.points[0].positions.resize(7);

		upper_joint_limits_.resize(7);
		lower_joint_limits_.resize(7);
		upper_joint_limits_[0] = 3.1416/180.0 * 170.0;
		lower_joint_limits_[0] = 3.1416/180.0 * -170.0;
		upper_joint_limits_[1] = 3.1416/180.0 * 120.0;
		lower_joint_limits_[1] = 3.1416/180.0 * -120.0;
		upper_joint_limits_[2] = 3.1416/180.0 * 170.0;
		lower_joint_limits_[2] = 3.1416/180.0 * -170.0;
		upper_joint_limits_[3] = 3.1416/180.0 * 120.0;
		lower_joint_limits_[3] = 3.1416/180.0 * -120.0;
		upper_joint_limits_[4] = 3.1416/180.0 * 170.0;
		lower_joint_limits_[4] = 3.1416/180.0 * -170.0;
		upper_joint_limits_[5] = 3.1416/180.0 * 120.0;
		lower_joint_limits_[5] = 3.1416/180.0 * -120.0;
		upper_joint_limits_[6] = 3.1416/180.0 * 175.0;
		lower_joint_limits_[6] = 3.1416/180.0 * -175.0;

    current_joint_positions_.resize(7);
		
  }

  void moveToBasePose() {
    planAndMove(base_pose_, std::string("base pose"));
  }
	
	// subscribes to the topic the joint angles are published into
  void registerSubscriberRelative(const std::string& topic) {
    joint_subscriber_ = node_handle_->subscribe(topic, 1, &JointFollower::jointCallbackRelative, this);
  }

  void registerSubscriberAbsolute(const std::string& topic) {
    joint_subscriber_ = node_handle_->subscribe(topic, 1, &JointFollower::jointCallbackAbsolute, this);
  }

  // void registerStateSubscriber(const std::string& topic) {
  //   state_subscriber_ = node_handle_->subscribe(topic, 1, &JointFollower::stateCallback, this);
  // }

  void setBasePose(const geometry_msgs::Pose& pose) {
    base_pose_ = pose;
  }

  geometry_msgs::Pose getBasePose() {
    return base_pose_;
  }
	
	void setBasePoseJointPositions(const std::vector<std::string>& names, const std::vector<double>& initial_angles) {
		// header missing?
		
		base_pose_joint_positions_.joint_names.resize(7);
		base_pose_joint_positions_.joint_names = names;
//		base_pose_joint_positions_.joint_names[0] = "iiwa_joint_1";
//		ROS_INFO_NAMED("joint_follower", "nach names1");
//		base_pose_joint_positions_.joint_names[2] = "iiwa_joint_3";
//		base_pose_joint_positions_.joint_names[3] = "iiwa_joint_4";
//		base_pose_joint_positions_.joint_names[4] = "iiwa_joint_5";
//		base_pose_joint_positions_.joint_names[5] = "iiwa_joint_6";
//		base_pose_joint_positions_.joint_names[6] = "iiwa_joint_7";
		base_pose_joint_positions_.points.resize(1);
		base_pose_joint_positions_.points[0].positions.resize(7);
		base_pose_joint_positions_.points[0].positions = initial_angles;
//		base_pose_joint_positions_.points[0].positions[0] = initial_angles[0];
//		base_pose_joint_positions_.points[0].positions[1] = initial_angles[1];
//		base_pose_joint_positions_.points[0].positions[2] = initial_angles[2];
//		base_pose_joint_positions_.points[0].positions[3] = initial_angles[3];
//		base_pose_joint_positions_.points[0].positions[4] = initial_angles[4];
//		base_pose_joint_positions_.points[0].positions[5] = initial_angles[5];
//		base_pose_joint_positions_.points[0].positions[6] = initial_angles[6];

		base_pose_joint_positions_.points[0].time_from_start = ros::Duration(0.01); // later variable
	}

	void moveToInitialJointPositions() {
		planAndMove(iiwa_initial_joint_positions_.points[0].positions, std::string("initial joint positions"));
	}
	
	void setBasePoseToCurrent() {
		base_pose_ = getPose(std::string("iiwa_link_ee")).pose;	
	}

  void createUncoupleService() {
    // service to couple/uncouple the joint follower  
    uncouple_service_ = node_handle_->advertiseService("coupler", &JointFollower::uncouple, this);
    ROS_INFO("Uncouple service created.");
  }

  void createJointPositionClientAndRequest() {
    joint_positions_client_ = node_handle_->serviceClient<joint_follower::GetJointPositions>("get_joint_positions", this);
    get_joint_positions_service_.request.joint_positions_req = true;
  }

private:
  ros::Subscriber joint_subscriber_;
	trajectory_msgs::JointTrajectory iiwa_initial_joint_positions_;
  geometry_msgs::Pose base_pose_;
	trajectory_msgs::JointTrajectory base_pose_joint_positions_; 
	trajectory_msgs::JointTrajectory mcs_initial_joint_positions_;
  double scale_factor_;
  double max_radius_;
  double max_radius2_;
	bool rad_input_;
	double angle_conversion_;
	bool first_time_;
	std::vector<double> upper_joint_limits_;
	std::vector<double> lower_joint_limits_;
	std::vector<double> direction_factor_; // TODO implement directions of counting via variable
  ros::ServiceServer uncouple_service_;
  bool coupled_;
  std::vector<double> current_joint_positions_;
  // ros::Subscriber state_subscriber_;
  ros::ServiceClient joint_positions_client_;
  joint_follower::GetJointPositions get_joint_positions_service_;

	// will get called when a new message has arrived on the subscribed topic
  void jointCallbackRelative(const iiwa_msgs::JointPosition::ConstPtr& msg) {
    if (coupled_) { 
			double a1 = msg->position.a1; // format -> jointAngles receive/read script
			double a2 = msg->position.a2;
			double a3 = msg->position.a3;
			double a4 = msg->position.a4;
			double a5 = msg->position.a5;
			double a6 = msg->position.a6;
			double a7 = msg->position.a7;

			if(first_time_) {
				mcs_initial_joint_positions_.points[0].positions[0] = a1;
				mcs_initial_joint_positions_.points[0].positions[1] = a2;
				mcs_initial_joint_positions_.points[0].positions[2] = a3;
				mcs_initial_joint_positions_.points[0].positions[3] = a4;
				mcs_initial_joint_positions_.points[0].positions[4] = a5; 
				mcs_initial_joint_positions_.points[0].positions[5] = a6; 
				mcs_initial_joint_positions_.points[0].positions[6] = a7;				
				first_time_ = false;
        ROS_INFO("Set initial joint positions.");
			}

      //geometry_msgs::Pose target_pose = base_pose_;
			trajectory_msgs::JointTrajectory trajectory_point = base_pose_joint_positions_;

      // In front. Mirrored.
			// trajectory_point.points[0].positions[0] -= (a1 - mcs_initial_joint_positions_.points[0].positions[0])*angle_conversion_;
			// trajectory_point.points[0].positions[1] -= (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
			// trajectory_point.points[0].positions[2] += (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
			// trajectory_point.points[0].positions[3] -= (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
			// trajectory_point.points[0].positions[4] += (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
			// trajectory_point.points[0].positions[5] -= (a7 - mcs_initial_joint_positions_.points[0].positions[6])*angle_conversion_;
			// trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

      // To the side. Not mirrored.
      trajectory_point.points[0].positions[0] += (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
      trajectory_point.points[0].positions[1] += (a1 - mcs_initial_joint_positions_.points[0].positions[0])*angle_conversion_;
      trajectory_point.points[0].positions[2] -= (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
      trajectory_point.points[0].positions[3] -= (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
      trajectory_point.points[0].positions[4] -= (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
      trajectory_point.points[0].positions[5] -= (a7 - mcs_initial_joint_positions_.points[0].positions[6])*angle_conversion_;
      trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

      // // Psychological experiment. Just q2 and q4
      // trajectory_point.points[0].positions[0] += 0.0*angle_conversion_;
      // trajectory_point.points[0].positions[1] += (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
      // trajectory_point.points[0].positions[2] -= 0.0*angle_conversion_;
      // trajectory_point.points[0].positions[3] += (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
      // trajectory_point.points[0].positions[4] -= 0.0*angle_conversion_;
      // trajectory_point.points[0].positions[5] += 0.0*angle_conversion_;
      // trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

      // No adaption for MCS. Just one by one.
      // trajectory_point.points[0].positions[0] += (a1 - mcs_initial_joint_positions_.points[0].positions[0])*angle_conversion_;
      // trajectory_point.points[0].positions[1] += (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
      // trajectory_point.points[0].positions[2] += (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
      // trajectory_point.points[0].positions[3] += (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
      // trajectory_point.points[0].positions[4] += (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
      // trajectory_point.points[0].positions[5] += (a6 - mcs_initial_joint_positions_.points[0].positions[5])*angle_conversion_;
      // trajectory_point.points[0].positions[6] += (a7 - mcs_initial_joint_positions_.points[0].positions[6])*angle_conversion_;



//			trajectory_point.points[0].positions[0] += (a1 - mcs_initial_joint_positions_.points[0].positions[0])*angle_conversion_;
//			trajectory_point.points[0].positions[1] += -1.0 * (a2 - mcs_initial_joint_positions_.points[0].positions[1])*angle_conversion_;
//			trajectory_point.points[0].positions[2] -= (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
//			trajectory_point.points[0].positions[3] += -1.0 * (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
//			trajectory_point.points[0].positions[4] -= (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
//			trajectory_point.points[0].positions[5] += (a7 - mcs_initial_joint_positions_.points[0].positions[6])*angle_conversion_;
//			trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

			// pick and place test
//			trajectory_point.points[0].positions[0] += (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
//			trajectory_point.points[0].positions[1] += 0.5 * (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
////			trajectory_point.points[0].positions[2] -= (a3 - mcs_initial_joint_positions_.points[0].positions[2])*angle_conversion_;
//			trajectory_point.points[0].positions[3] += (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
////			trajectory_point.points[0].positions[4] -= (a5 - mcs_initial_joint_positions_.points[0].positions[4])*angle_conversion_;
//			trajectory_point.points[0].positions[5] += 0.5 * (a4 - mcs_initial_joint_positions_.points[0].positions[3])*angle_conversion_;
////			trajectory_point.points[0].positions[6] += 0.0*angle_conversion_;

			trajectory_point = jointLimitation(trajectory_point);

      publishTrajectory(trajectory_point);
//    }
    }
  }

  void jointCallbackAbsolute(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    publishPoseGoal(msg->pose, 0.01);
  }

	trajectory_msgs::JointTrajectory jointLimitation(trajectory_msgs::JointTrajectory set_value) {
		for (int i=0; i<7; i++) {
			if(set_value.points[0].positions[i] > (upper_joint_limits_[i]-(3.1416/180 * 2.0))) {
				set_value.points[0].positions[i] = upper_joint_limits_[i]-(3.1416/180 * 2.0);
			}
			else if	(set_value.points[0].positions[i] < (lower_joint_limits_[i]+(3.1416/180 * 2.0))) {
				set_value.points[0].positions[i] = lower_joint_limits_[i]+(3.1416/180 * 2.0);
			}		
		}
		return set_value;   
	}

  bool uncouple(joint_follower::Coupler::Request &req, joint_follower::Coupler::Response &res) {
    if (req.uncouple_req) {
      coupled_ = false;
      first_time_ = true;
      if (joint_positions_client_.call(get_joint_positions_service_)) {
        current_joint_positions_[0] = get_joint_positions_service_.response.q1;
        current_joint_positions_[1] = get_joint_positions_service_.response.q2;
        current_joint_positions_[2] = get_joint_positions_service_.response.q3;
        current_joint_positions_[3] = get_joint_positions_service_.response.q4;
        current_joint_positions_[4] = get_joint_positions_service_.response.q5;
        current_joint_positions_[5] = get_joint_positions_service_.response.q6;
        current_joint_positions_[6] = get_joint_positions_service_.response.q7;
        setBasePoseJointPositions(getJointNames(), current_joint_positions_);
        ROS_INFO("yuuuuuurp");
      }
      else {
        ROS_INFO("Failed to call service get_joint_positions");
      }
      // setBasePoseJointPositions(getJointNames(), getJointPositions());
      // setBasePoseJointPositions(getJointNames(), current_joint_positions_);
      res.uncouple_res = "Uncoupled!";
      ROS_INFO("Uncoupled!");
    }
    else if (!req.uncouple_req) {
      coupled_ = true;
      res.uncouple_res = "Coupled!";
      ROS_INFO("Coupled!");
    }
    return true;
  }

//   void stateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//     current_joint_positions_ = msg->position;
// //    ROS_INFO("%.4f %.4f %.4f %.4f %.4f %.4f %.4f", current_joint_positions_[0], current_joint_positions_[1], current_joint_positions_[2], current_joint_positions_[3], current_joint_positions_[4], current_joint_positions_[5], current_joint_positions_[6]);  
//   }
	
};
} // namespace joint_follower

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_joint_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

//	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("pose", 1000);
//	geometry_msgs::PoseStamped current_pose;

	std::vector<double> initial_joint_positions;
	std::vector<std::string> joint_names;
	
	double scale_factor; // MOD
	node_handle.param("/iiwa/joint_follower/scale_factor", scale_factor, 1.0);
	
	// input in rad or deg
	bool rad_input;
	node_handle.param("/iiwa/joint_follower/rad_input", rad_input, true);

	// input from udp or csv
	bool udp_input;
	node_handle.param("/iiwa/joint_follower/udp", udp_input, false);


  joint_follower::JointFollower joint_follower(&node_handle, "manipulator", "world", scale_factor, 2, rad_input);

  // joint_follower.registerStateSubscriber(std::string("/iiwa/joint_states")); 
	joint_follower.createUncoupleService();

  // Create service client to request the joint positions only when needed
  joint_follower.createJointPositionClientAndRequest();

//	// use when base pose is given
//  joint_follower.moveToBasePose();
//	joint_names = joint_follower.getJointNames();
//	initial_joint_positions = joint_follower.getJointPositions();
//	joint_follower.setBasePoseJointPositions(joint_names, initial_joint_positions);

	// use when initial joint positions are given
	joint_follower.moveToInitialJointPositions();
	//joint_follower.setBasePoseToCurrent();		
	joint_names = joint_follower.getJointNames();
	initial_joint_positions = joint_follower.getJointPositions();
	joint_follower.setBasePoseJointPositions(joint_names, initial_joint_positions);

  joint_follower.waitForApproval();
	if(udp_input) {
		joint_follower.registerSubscriberRelative(std::string("/iiwa/jointAnglesFromUDP/JointPosition"));
		ROS_INFO_NAMED("joint_follower", "Subscribed to set of joint angles from UDP!");
	}
	else {
  	joint_follower.registerSubscriberRelative(std::string("/jointAnglesFromFile/JointPosition"));
		ROS_INFO_NAMED("joint_follower", "Subscribed to set of joint angles from file!");
	} 
//  joint_follower.registerSubscriberRelative(std::string("/jointAnglesFromUDP/JointPosition"));
  //joint_follower.registerSubscriberAbsolute(std::string("/jointFromFile/PoseStampedAbsolute"));

//  if(joint_follower.setJointImpedanceMode(0.7, 0.3)) {
//		ROS_INFO_NAMED("joint_follower", "Success");
//	}
//	else {
//		ROS_INFO_NAMED("joint_follower", "Failure");
//	}

  ros::Rate rate(100);
  while(ros::ok()) {
//		current_pose = joint_follower.getPose(std::string("iiwa_link_ee"));
//		//ROS_INFO("%s", std::to_string(current_pose.pose.position.x).c_str());
//		pose_pub.publish(current_pose);
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
