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
#include <iiwa_msgs/JointPosition.h> // how to include msg type of iiwa stack? JointQuantity also needed?

//TODO not only use positions, use speed and accelerations too
//TODO try out using moveIt! planning live

namespace joint_follower {


class JointFollower : public iimoveit::RobotInterface {
public:
  JointFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame, double scale_factor, double max_radius)
      : RobotInterface(node_handle, planning_group, base_frame),
        scale_factor_(scale_factor),
        max_radius_(max_radius),
        max_radius2_(max_radius*max_radius) {
    //base_pose_.position.x = 0.5;
    //base_pose_.position.y = 0.0;
    //base_pose_.position.z = 0.6;
    //base_pose_.orientation.x = 0.0;
    //base_pose_.orientation.y = 1.0;
    //base_pose_.orientation.z = 0.0;
    //base_pose_.orientation.w = 0.0;
		//base_pose_.position.x = -0.3; //changed base pose 
    //base_pose_.position.y = 0.5;
    //base_pose_.position.z = 0.6;
		base_pose_.position.x = -0.5; //changed base pose -> base in shoulder
    base_pose_.position.y = 0.3;
    base_pose_.position.z = 0.6;
    base_pose_.orientation.x = -sqrt(2.0)/2.0;
    base_pose_.orientation.y = 0.0;
    base_pose_.orientation.z = 0.0;
    base_pose_.orientation.w = sqrt(2.0)/2.0;
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

  void setBasePose(const geometry_msgs::Pose& pose) {
    base_pose_ = pose;
  }

  geometry_msgs::Pose getBasePose() {
    return base_pose_;
  }
	
	void setBasePoseJointPositions(const std::vector<std::string>& names, const std::vector<double>& initial_angles) {
		// header + names + time_from_start missing?
		
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


private:
  ros::Subscriber joint_subscriber_;
  geometry_msgs::Pose base_pose_;
	trajectory_msgs::JointTrajectory base_pose_joint_positions_; 
  double scale_factor_;
  double max_radius_;
  double max_radius2_;

	// will get called when a new message has arrived on the subscribed topic
  void jointCallbackRelative(const iiwa_msgs::JointPosition::ConstPtr& msg) { // ConstPtr&? -> typedef constant pointer
			double a1 = msg->position.a1; // format -> jointAngles receive/read script
			double a2 = msg->position.a2;
			double a3 = msg->position.a3;
			double a4 = msg->position.a4;
			double a5 = msg->position.a5;
			double a6 = msg->position.a6;
			double a7 = msg->position.a7;
//    double x = msg->pose.position.x * scale_factor_;
//    double y = msg->pose.position.y * scale_factor_;
//    double z = msg->pose.position.z * scale_factor_;
//    if (x*x + y*y + z*z <= max_radius2_) {
//      tf::Quaternion base_quaternion(base_pose_.orientation.x, base_pose_.orientation.y, base_pose_.orientation.z, base_pose_.orientation.w);
//      tf::Quaternion next_quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
//      tf::Quaternion result_quaternion = next_quaternion * base_quaternion;

      //geometry_msgs::Pose target_pose = base_pose_;
			trajectory_msgs::JointTrajectory trajectory_point = base_pose_joint_positions_; // initial joint positions? -> RI: getJointPositions()
			// format JointTracectoryPoint? float64[] positions (float64 -> double in C++)
			trajectory_point.points[0].positions[0] += a1;
			trajectory_point.points[0].positions[1] += a2;
			trajectory_point.points[0].positions[2] += a3;
			trajectory_point.points[0].positions[3] += a4;
			trajectory_point.points[0].positions[4] += a5;
			trajectory_point.points[0].positions[5] += a6;
			trajectory_point.points[0].positions[6] += a7;
//      target_pose.position.x += x;
//      target_pose.position.y += y;
//      target_pose.position.z += z;
//      target_pose.orientation.x = result_quaternion.getX();
//      target_pose.orientation.y = result_quaternion.getY();
//      target_pose.orientation.z = result_quaternion.getZ();
//      target_pose.orientation.w = result_quaternion.getW();
      publishTrajectory(trajectory_point);
//    }
  }

  void jointCallbackAbsolute(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    publishPoseGoal(msg->pose, 0.01);
  }
};
} // namespace joint_follower

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_joint_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	std::vector<double> initial_joint_positions;
	std::vector<std::string> joint_names;
	
	double scale_factor; // MOD
	node_handle.param("/iiwa/joint_follower/scale_factor", scale_factor, 1.0);

  joint_follower::JointFollower joint_follower(&node_handle, "manipulator", "world", scale_factor, 2);
  joint_follower.moveToBasePose();
	// get initial joint positions
	ROS_INFO_NAMED("joint_follower", "vor getJointPositions()");
	joint_names = joint_follower.getJointNames();
	initial_joint_positions = joint_follower.getJointPositions();
	ROS_INFO_NAMED("joint_follower", "nach getJointPositions()"); 
	joint_follower.setBasePoseJointPositions(joint_names, initial_joint_positions);
	ROS_INFO_NAMED("joint_follower", "nach setBasePoseJointPositions()");
  joint_follower.waitForApproval();
  joint_follower.registerSubscriberRelative(std::string("/jointAnglesFromFile/JointPositionRelative")); // jointAngles_read.py has to be written
  //joint_follower.registerSubscriberAbsolute(std::string("/jointFromFile/PoseStampedAbsolute"));
  ROS_INFO_NAMED("joint_follower", "Subscribed to set of joint angles!");


  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
