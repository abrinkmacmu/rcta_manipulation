#ifndef __narms_utils__
#define __narms_utils__
#include <random>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetPositionIK.h>

namespace
{
	std::mt19937 mt(1729);
}

double sample_from_gaussian(double mean, double variance);

void printPose(geometry_msgs::Pose p);

tf::Transform geoPose2Transform(geometry_msgs::Pose p);

geometry_msgs::Pose convertPoseViaTransform( 
	const geometry_msgs::Pose& pose, 
const tf::StampedTransform& tf);

geometry_msgs::Pose generatePose(double x, double y, double z, double roll, double pitch, double yaw);

geometry_msgs::Pose generateRandomPose();


 void getIKServerRequest(geometry_msgs::Pose pose, std::string planning_group, moveit_msgs::GetPositionIK& ik_srv);

 void getGraspingPoses(geometry_msgs::Pose input, geometry_msgs::Pose pr2_pose, geometry_msgs::Pose roman_pose);

#endif