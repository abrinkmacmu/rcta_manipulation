#ifndef __states__
#define __states__

#include <string>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

bool checkStartandGoalIK(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, 
	ros::ServiceClient& computeIK,
	std::string& startRobot, std::string& goalRobot);

bool computeSampledHandoffPose(geometry_msgs::Pose startPose, 
								geometry_msgs::Pose goalPose,
							std::string startRobot, 
							std::string goalRobot, 
							geometry_msgs::Pose& handoffPose,
							unsigned int num_of_handoff_samples);

#endif