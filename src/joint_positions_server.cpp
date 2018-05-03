#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <iimoveit/robot_interface.h>
#include <joint_follower/GetJointPositions.h>

std_msgs::Float64 q1, q2, q3, q4, q5, q6, q7;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  q1.data = msg->position[0];
  q2.data = msg->position[1];
  q3.data = msg->position[2];
  q4.data = msg->position[3];
  q5.data = msg->position[4];
  q6.data = msg->position[5];
  q7.data = msg->position[6];
}

bool sendJointPositions(joint_follower::GetJointPositions::Request &req, joint_follower::GetJointPositions::Response &res) {
  res.q1 = q1.data;
  res.q2 = q2.data;
  res.q3 = q3.data;
  res.q4 = q4.data;
  res.q5 = q5.data;
  res.q6 = q6.data;
  res.q7 = q7.data;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_positions_server");

  ros::NodeHandle node_handle;
	
  ros::AsyncSpinner spinner(1);
  spinner.start();	

  ros::ServiceServer joint_positions_service = node_handle.advertiseService("get_joint_positions", sendJointPositions);
  ros::Subscriber joint_states_sub = node_handle.subscribe("/iiwa/joint_states", 1, jointStatesCallback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    loop_rate.sleep();
  }
	ros::shutdown();
  return 0;
}
