#ifndef __narms_utils__
#define __narms_utils__
#include <random>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/CollisionObject.h>

namespace
{
	std::mt19937 mt(1729);
}

double sample_from_gaussian(double mean, double variance);

void printPose(geometry_msgs::Pose p);

tf::Transform geoPose2Transform(geometry_msgs::Pose p);

geometry_msgs::Pose Affine2Pose(Eigen::Affine3d A);

geometry_msgs::Pose convertPoseViaTransform( 
	const geometry_msgs::Pose& pose, 
const tf::Transform& tf);

geometry_msgs::Pose generatePose(double x, double y, double z, double roll, double pitch, double yaw);

geometry_msgs::Pose generateRandomPose();


 void getIKServerRequest(geometry_msgs::Pose pose, std::string planning_group, moveit_msgs::GetPositionIK& ik_srv);

 void getGraspingPoses(const tf::StampedTransform& tf_object, geometry_msgs::Pose& pr2_pose, geometry_msgs::Pose& roman_pose);

 moveit_msgs::CollisionObject createCollisionBox(
	geometry_msgs::Pose object_pose,
	std::string reference_name,
	std::string tf_frame, 
	double sx, double sy, double sz);

#endif