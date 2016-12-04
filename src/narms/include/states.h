#ifndef __states__
#define __states__

#include <string>
#include <numeric>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <narms_utils.h>
#include <narms/target_pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

struct combinedTrajectories
{
	geometry_msgs::Pose robot1_grasp;
	geometry_msgs::Pose robot2_grasp;
};
bool checkStartandGoalIK(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, 
	ros::ServiceClient& computeIK,
	std::string& startRobot, std::string& goalRobot);

bool computeSampledHandoffPose(geometry_msgs::Pose startPose, 
								geometry_msgs::Pose goalPose, 
								std::string startRobot,
								std::string goalRobot, 
								geometry_msgs::Pose& handoffPose,
								unsigned int num_of_handoff_samples,
								ros::ServiceClient& compute_ik,
								ros::ServiceClient& robot1_move_arm_server,
								ros::ServiceClient& robot2_move_arm_server,
								double planning_time);


// double score_trajectory(moveit_msgs::RobotTrajectory& traj);
double score_trajectory(std::vector<trajectory_msgs::JointTrajectoryPoint>& traj);

#endif